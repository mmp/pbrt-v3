
/*
    pbrt source code is Copyright(c) 1998-2016
                        Matt Pharr, Greg Humphreys, and Wenzel Jakob.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

// core/parser.cpp*
#include "parser.h"
#include "api.h"
#include "fileutil.h"
#include "memory.h"
#include "paramset.h"
#include "stats.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#ifdef PBRT_HAVE_MMAP
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#elif defined(PBRT_IS_WINDOWS)
#include <windows.h>  // Windows file mapping API
#endif
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace pbrt {

Loc *parserLoc;

static std::string toString(string_view s) {
    return std::string(s.data(), s.size());
}

STAT_MEMORY_COUNTER("Memory/Tokenizer buffers", tokenizerMemory);

static char decodeEscaped(int ch) {
    switch (ch) {
    case EOF:
        Error("premature EOF after character escape '\\'");
        exit(1);
    case 'b':
        return '\b';
    case 'f':
        return '\f';
    case 'n':
        return '\n';
    case 'r':
        return '\r';
    case 't':
        return '\t';
    case '\\':
        return '\\';
    case '\'':
        return '\'';
    case '\"':
        return '\"';
    default:
        Error("unexpected escaped character \"%c\"", ch);
        exit(1);
    }
    return 0;  // NOTREACHED
}

std::unique_ptr<Tokenizer> Tokenizer::CreateFromFile(
    const std::string &filename,
    std::function<void(const char *)> errorCallback) {
    if (filename == "-") {
        // Handle stdin by slurping everything into a string.
        std::string str;
        int ch;
        while ((ch = getchar()) != EOF) str.push_back((char)ch);
        // std::make_unique...
        return std::unique_ptr<Tokenizer>(
            new Tokenizer(std::move(str), std::move(errorCallback)));
    }

#ifdef PBRT_HAVE_MMAP
    int fd = open(filename.c_str(), O_RDONLY);
    if (fd == -1) {
        errorCallback(
            StringPrintf("%s: %s", filename.c_str(), strerror(errno)).c_str());
        return nullptr;
    }

    struct stat stat;
    if (fstat(fd, &stat) != 0) {
        errorCallback(
            StringPrintf("%s: %s", filename.c_str(), strerror(errno)).c_str());
        return nullptr;
    }

    size_t len = stat.st_size;
    void *ptr = mmap(0, len, PROT_READ, MAP_FILE | MAP_SHARED, fd, 0);
    if (close(fd) != 0) {
        errorCallback(
            StringPrintf("%s: %s", filename.c_str(), strerror(errno)).c_str());
        return nullptr;
    }

    // return std::make_unique<Tokenizer>(ptr, len);
    return std::unique_ptr<Tokenizer>(
        new Tokenizer(ptr, len, filename, std::move(errorCallback)));
#elif defined(PBRT_IS_WINDOWS)
    auto errorReportLambda = [&errorCallback,
                              &filename]() -> std::unique_ptr<Tokenizer> {
        LPSTR messageBuffer = nullptr;
        FormatMessageA(
            FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                FORMAT_MESSAGE_IGNORE_INSERTS,
            NULL, ::GetLastError(), MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
            (LPSTR)&messageBuffer, 0, NULL);

        errorCallback(
            StringPrintf("%s: %s", filename.c_str(), messageBuffer).c_str());

        LocalFree(messageBuffer);
        return nullptr;
    };

    HANDLE fileHandle =
        CreateFileA(filename.c_str(), GENERIC_READ, FILE_SHARE_READ, 0,
                    OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
    if (!fileHandle) {
        return errorReportLambda();
    }

    LARGE_INTEGER liLen;
    if (!GetFileSizeEx(fileHandle, &liLen)) {
        return errorReportLambda();
    }
    size_t len = liLen.QuadPart;

    HANDLE mapping = CreateFileMapping(fileHandle, 0, PAGE_READONLY, 0, 0, 0);
    CloseHandle(fileHandle);
    if (mapping == 0) {
        return errorReportLambda();
    }

    LPVOID ptr = MapViewOfFile(mapping, FILE_MAP_READ, 0, 0, 0);
    CloseHandle(mapping);
    if (ptr == nullptr) {
        return errorReportLambda();
    }

    return std::unique_ptr<Tokenizer>(
        new Tokenizer(ptr, len, filename, std::move(errorCallback)));
#else
    FILE *f = fopen(filename.c_str(), "r");
    if (!f) {
        errorCallback(
            StringPrintf("%s: %s", filename.c_str(), strerror(errno)).c_str());
        return nullptr;
    }

    std::string str;
    int ch;
    while ((ch = fgetc(f)) != EOF) str.push_back(char(ch));
    fclose(f);

    // std::make_unique...
    return std::unique_ptr<Tokenizer>(
        new Tokenizer(std::move(str), std::move(errorCallback)));
#endif
}

std::unique_ptr<Tokenizer> Tokenizer::CreateFromString(
    std::string str, std::function<void(const char *)> errorCallback) {
    // return std::make_unique<Tokenizer>(std::move(str));
    return std::unique_ptr<Tokenizer>(
        new Tokenizer(std::move(str), std::move(errorCallback)));
}

Tokenizer::Tokenizer(std::string str,
                     std::function<void(const char *)> errorCallback)
    : loc("<stdin>"),
      errorCallback(std::move(errorCallback)),
      contents(std::move(str)) {
    pos = contents.data();
    end = pos + contents.size();
    tokenizerMemory += contents.size();
}

#if defined(PBRT_HAVE_MMAP) || defined(PBRT_IS_WINDOWS)
Tokenizer::Tokenizer(void *ptr, size_t len, std::string filename,
                     std::function<void(const char *)> errorCallback)
    : loc(filename),
      errorCallback(std::move(errorCallback)),
      unmapPtr(ptr),
      unmapLength(len) {
    pos = (const char *)ptr;
    end = pos + len;
}
#endif

Tokenizer::~Tokenizer() {
#ifdef PBRT_HAVE_MMAP
    if (unmapPtr && unmapLength > 0)
        if (munmap(unmapPtr, unmapLength) != 0)
            errorCallback(StringPrintf("munmap: %s", strerror(errno)).c_str());
#elif defined(PBRT_IS_WINDOWS)
    if (unmapPtr) {
        if (UnmapViewOfFile(unmapPtr) == 0) {
            LPSTR messageBuffer = nullptr;
            FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER |
                               FORMAT_MESSAGE_FROM_SYSTEM |
                               FORMAT_MESSAGE_IGNORE_INSERTS,
                           NULL, ::GetLastError(),
                           MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                           (LPSTR)&messageBuffer, 0, NULL);
            errorCallback(
                StringPrintf("UnmapViewOfFile: %s", messageBuffer).c_str());
            LocalFree(messageBuffer);
        }
    }
#endif
}

string_view Tokenizer::Next() {
    while (true) {
        const char *tokenStart = pos;
        int ch = getChar();
        if (ch == EOF)
            return {};
        else if (ch == ' ' || ch == '\n' || ch == '\t' || ch == '\r') {
            // nothing
        } else if (ch == '"') {
            // scan to closing quote
            bool haveEscaped = false;
            while ((ch = getChar()) != '"') {
                if (ch == EOF) {
                    errorCallback("premature EOF");
                    return {};
                } else if (ch == '\n') {
                    errorCallback("unterminated string");
                    return {};
                } else if (ch == '\\') {
                    haveEscaped = true;
                    // Grab the next character
                    if ((ch = getChar()) == EOF) {
                        errorCallback("premature EOF");
                        return {};
                    }
                }
            }

            if (!haveEscaped)
                return {tokenStart, size_t(pos - tokenStart)};
            else {
                sEscaped.clear();
                for (const char *p = tokenStart; p < pos; ++p) {
                    if (*p != '\\')
                        sEscaped.push_back(*p);
                    else {
                        ++p;
                        CHECK_LT(p, pos);
                        sEscaped.push_back(decodeEscaped(*p));
                    }
                }
                return {sEscaped.data(), sEscaped.size()};
            }
        } else if (ch == '[' || ch == ']') {
            return {tokenStart, size_t(1)};
        } else if (ch == '#') {
            // comment: scan to EOL (or EOF)
            while ((ch = getChar()) != EOF) {
                if (ch == '\n' || ch == '\r') {
                    ungetChar();
                    break;
                }
            }

            return {tokenStart, size_t(pos - tokenStart)};
        } else {
            // Regular statement or numeric token; scan until we hit a
            // space, opening quote, or bracket.
            while ((ch = getChar()) != EOF) {
                if (ch == ' ' || ch == '\n' || ch == '\t' || ch == '\r' ||
                    ch == '"' || ch == '[' || ch == ']') {
                    ungetChar();
                    break;
                }
            }
            return {tokenStart, size_t(pos - tokenStart)};
        }
    }
}

static double parseNumber(string_view str) {
    // Fast path for a single digit
    if (str.size() == 1) {
        if (!(str[0] >= '0' && str[0] <= '9')) {
            Error("\"%c\": expected a number", str[0]);
            exit(1);
        }
        return str[0] - '0';
    }

    // Copy to a buffer so we can NUL-terminate it, as strto[idf]() expect.
    char buf[64];
    char *bufp = buf;
    std::unique_ptr<char[]> allocBuf;
    if (str.size() + 1 >= sizeof(buf)) {
        // This should be very unusual, but is necessary in case we get a
        // goofball number with lots of leading zeros, for example.
        allocBuf.reset(new char[str.size() + 1]);
        bufp = allocBuf.get();
    }

    std::copy(str.begin(), str.end(), bufp);
    bufp[str.size()] = '\0';

    // Can we just use strtol?
    auto isInteger = [](string_view str) {
        for (char ch : str)
            if (!(ch >= '0' && ch <= '9')) return false;
        return true;
    };

    char *endptr = nullptr;
    double val;
    if (isInteger(str))
        val = double(strtol(bufp, &endptr, 10));
    else if (sizeof(Float) == sizeof(float))
        val = strtof(bufp, &endptr);
    else
        val = strtod(bufp, &endptr);

    if (val == 0 && endptr == bufp) {
        Error("%s: expected a number", toString(str).c_str());
        exit(1);
    }

    return val;
}

inline bool isQuotedString(string_view str) {
    return str.size() >= 2 && str[0] == '"' && str.back() == '"';
}

static string_view dequoteString(string_view str) {
    if (!isQuotedString(str)) {
        Error("\"%s\": expected quoted string", toString(str).c_str());
        exit(1);
    }

    str.remove_prefix(1);
    str.remove_suffix(1);
    return str;
}

struct ParamListItem {
    std::string name;
    double *doubleValues = nullptr;
    const char **stringValues = nullptr;
    size_t size = 0;
    bool isString = false;
};

PBRT_CONSTEXPR int TokenOptional = 0;
PBRT_CONSTEXPR int TokenRequired = 1;

enum {
    PARAM_TYPE_INT,
    PARAM_TYPE_BOOL,
    PARAM_TYPE_FLOAT,
    PARAM_TYPE_POINT2,
    PARAM_TYPE_VECTOR2,
    PARAM_TYPE_POINT3,
    PARAM_TYPE_VECTOR3,
    PARAM_TYPE_NORMAL,
    PARAM_TYPE_RGB,
    PARAM_TYPE_XYZ,
    PARAM_TYPE_BLACKBODY,
    PARAM_TYPE_SPECTRUM,
    PARAM_TYPE_STRING,
    PARAM_TYPE_TEXTURE
};

static bool lookupType(const std::string &decl, int *type, std::string &sname) {
    *type = 0;
    // Skip leading space
    auto skipSpace = [&decl](std::string::const_iterator iter) {
        while (iter != decl.end() && (*iter == ' ' || *iter == '\t')) ++iter;
        return iter;
    };
    // Skip to the next whitespace character (or the end of the string).
    auto skipToSpace = [&decl](std::string::const_iterator iter) {
        while (iter != decl.end() && *iter != ' ' && *iter != '\t') ++iter;
        return iter;
    };

    auto typeBegin = skipSpace(decl.begin());
    if (typeBegin == decl.end()) {
        Error("Parameter \"%s\" doesn't have a type declaration?!",
              decl.c_str());
        return false;
    }

    // Find end of type declaration
    auto typeEnd = skipToSpace(typeBegin);

    string_view typeStr(&(*typeBegin), size_t(typeEnd - typeBegin));
    if (typeStr == "float")
        *type = PARAM_TYPE_FLOAT;
    else if (typeStr == "integer")
        *type = PARAM_TYPE_INT;
    else if (typeStr == "bool")
        *type = PARAM_TYPE_BOOL;
    else if (typeStr == "point2")
        *type = PARAM_TYPE_POINT2;
    else if (typeStr == "vector2")
        *type = PARAM_TYPE_VECTOR2;
    else if (typeStr == "point3")
        *type = PARAM_TYPE_POINT3;
    else if (typeStr == "vector3")
        *type = PARAM_TYPE_VECTOR3;
    else if (typeStr == "point")
        *type = PARAM_TYPE_POINT3;
    else if (typeStr == "vector")
        *type = PARAM_TYPE_VECTOR3;
    else if (typeStr == "normal")
        *type = PARAM_TYPE_NORMAL;
    else if (typeStr == "string")
        *type = PARAM_TYPE_STRING;
    else if (typeStr == "texture")
        *type = PARAM_TYPE_TEXTURE;
    else if (typeStr == "color")
        *type = PARAM_TYPE_RGB;
    else if (typeStr == "rgb")
        *type = PARAM_TYPE_RGB;
    else if (typeStr == "xyz")
        *type = PARAM_TYPE_XYZ;
    else if (typeStr == "blackbody")
        *type = PARAM_TYPE_BLACKBODY;
    else if (typeStr == "spectrum")
        *type = PARAM_TYPE_SPECTRUM;
    else {
        Error("Unable to decode type from \"%s\"", decl.c_str());
        return false;
    }

    auto nameBegin = skipSpace(typeEnd);
    if (nameBegin == decl.end()) {
        Error("Unable to find parameter name from \"%s\"", decl.c_str());
        return false;
    }
    auto nameEnd = skipToSpace(nameBegin);
    sname = std::string(nameBegin, nameEnd);

    return true;
}

static const char *paramTypeToName(int type) {
    switch (type) {
    case PARAM_TYPE_INT:
        return "int";
    case PARAM_TYPE_BOOL:
        return "bool";
    case PARAM_TYPE_FLOAT:
        return "float";
    case PARAM_TYPE_POINT2:
        return "point2";
    case PARAM_TYPE_VECTOR2:
        return "vector2";
    case PARAM_TYPE_POINT3:
        return "point3";
    case PARAM_TYPE_VECTOR3:
        return "vector3";
    case PARAM_TYPE_NORMAL:
        return "normal";
    case PARAM_TYPE_RGB:
        return "rgb/color";
    case PARAM_TYPE_XYZ:
        return "xyz";
    case PARAM_TYPE_BLACKBODY:
        return "blackbody";
    case PARAM_TYPE_SPECTRUM:
        return "spectrum";
    case PARAM_TYPE_STRING:
        return "string";
    case PARAM_TYPE_TEXTURE:
        return "texture";
    default:
        LOG(FATAL) << "Error in paramTypeToName";
        return nullptr;
    }
}

static void AddParam(ParamSet &ps, const ParamListItem &item,
                     SpectrumType spectrumType) {
    int type;
    std::string name;
    if (lookupType(item.name, &type, name)) {
        if (type == PARAM_TYPE_TEXTURE || type == PARAM_TYPE_STRING ||
            type == PARAM_TYPE_BOOL) {
            if (!item.stringValues) {
                Error(
                    "Expected string parameter value for parameter "
                    "\"%s\" with type \"%s\". Ignoring.",
                    name.c_str(), paramTypeToName(type));
                return;
            }
        } else if (type !=
                   PARAM_TYPE_SPECTRUM) { /* spectrum can be either... */
            if (item.stringValues) {
                Error(
                    "Expected numeric parameter value for parameter "
                    "\"%s\" with type \"%s\".  Ignoring.",
                    name.c_str(), paramTypeToName(type));
                return;
            }
        }

        int nItems = item.size;
        if (type == PARAM_TYPE_INT) {
            // parser doesn't handle ints, so convert from doubles here....
            int nAlloc = nItems;
            std::unique_ptr<int[]> idata(new int[nAlloc]);
            for (int j = 0; j < nAlloc; ++j)
                idata[j] = int(item.doubleValues[j]);
            ps.AddInt(name, std::move(idata), nItems);
        } else if (type == PARAM_TYPE_BOOL) {
            // strings -> bools
            int nAlloc = item.size;
            std::unique_ptr<bool[]> bdata(new bool[nAlloc]);
            for (int j = 0; j < nAlloc; ++j) {
                std::string s(item.stringValues[j]);
                if (s == "true")
                    bdata[j] = true;
                else if (s == "false")
                    bdata[j] = false;
                else {
                    Warning(
                        "Value \"%s\" unknown for Boolean parameter \"%s\"."
                        "Using \"false\".",
                        s.c_str(), item.name.c_str());
                    bdata[j] = false;
                }
            }
            ps.AddBool(name, std::move(bdata), nItems);
        } else if (type == PARAM_TYPE_FLOAT) {
            std::unique_ptr<Float[]> floats(new Float[nItems]);
            for (int i = 0; i < nItems; ++i) floats[i] = item.doubleValues[i];
            ps.AddFloat(name, std::move(floats), nItems);
        } else if (type == PARAM_TYPE_POINT2) {
            if ((nItems % 2) != 0)
                Warning(
                    "Excess values given with point2 parameter \"%s\". "
                    "Ignoring last one of them.",
                    item.name.c_str());
            std::unique_ptr<Point2f[]> pts(new Point2f[nItems / 2]);
            for (int i = 0; i < nItems / 2; ++i) {
                pts[i].x = item.doubleValues[2 * i];
                pts[i].y = item.doubleValues[2 * i + 1];
            }
            ps.AddPoint2f(name, std::move(pts), nItems / 2);
        } else if (type == PARAM_TYPE_VECTOR2) {
            if ((nItems % 2) != 0)
                Warning(
                    "Excess values given with vector2 parameter \"%s\". "
                    "Ignoring last one of them.",
                    item.name.c_str());
            std::unique_ptr<Vector2f[]> vecs(new Vector2f[nItems / 2]);
            for (int i = 0; i < nItems / 2; ++i) {
                vecs[i].x = item.doubleValues[2 * i];
                vecs[i].y = item.doubleValues[2 * i + 1];
            }
            ps.AddVector2f(name, std::move(vecs), nItems / 2);
        } else if (type == PARAM_TYPE_POINT3) {
            if ((nItems % 3) != 0)
                Warning(
                    "Excess values given with point3 parameter \"%s\". "
                    "Ignoring last %d of them.",
                    item.name.c_str(), nItems % 3);
            std::unique_ptr<Point3f[]> pts(new Point3f[nItems / 3]);
            for (int i = 0; i < nItems / 3; ++i) {
                pts[i].x = item.doubleValues[3 * i];
                pts[i].y = item.doubleValues[3 * i + 1];
                pts[i].z = item.doubleValues[3 * i + 2];
            }
            ps.AddPoint3f(name, std::move(pts), nItems / 3);
        } else if (type == PARAM_TYPE_VECTOR3) {
            if ((nItems % 3) != 0)
                Warning(
                    "Excess values given with vector3 parameter \"%s\". "
                    "Ignoring last %d of them.",
                    item.name.c_str(), nItems % 3);
            std::unique_ptr<Vector3f[]> vecs(new Vector3f[nItems / 3]);
            for (int j = 0; j < nItems / 3; ++j) {
                vecs[j].x = item.doubleValues[3 * j];
                vecs[j].y = item.doubleValues[3 * j + 1];
                vecs[j].z = item.doubleValues[3 * j + 2];
            }
            ps.AddVector3f(name, std::move(vecs), nItems / 3);
        } else if (type == PARAM_TYPE_NORMAL) {
            if ((nItems % 3) != 0)
                Warning(
                    "Excess values given with \"normal\" parameter \"%s\". "
                    "Ignoring last %d of them.",
                    item.name.c_str(), nItems % 3);
            std::unique_ptr<Normal3f[]> normals(new Normal3f[nItems / 3]);
            for (int j = 0; j < nItems / 3; ++j) {
                normals[j].x = item.doubleValues[3 * j];
                normals[j].y = item.doubleValues[3 * j + 1];
                normals[j].z = item.doubleValues[3 * j + 2];
            }
            ps.AddNormal3f(name, std::move(normals), nItems / 3);
        } else if (type == PARAM_TYPE_RGB) {
            if ((nItems % 3) != 0) {
                Warning(
                    "Excess RGB values given with parameter \"%s\". "
                    "Ignoring last %d of them",
                    item.name.c_str(), nItems % 3);
                nItems -= nItems % 3;
            }
            std::unique_ptr<Float[]> floats(new Float[nItems]);
            for (int j = 0; j < nItems; ++j) floats[j] = item.doubleValues[j];
            ps.AddRGBSpectrum(name, std::move(floats), nItems);
        } else if (type == PARAM_TYPE_XYZ) {
            if ((nItems % 3) != 0) {
                Warning(
                    "Excess XYZ values given with parameter \"%s\". "
                    "Ignoring last %d of them",
                    item.name.c_str(), nItems % 3);
                nItems -= nItems % 3;
            }
            std::unique_ptr<Float[]> floats(new Float[nItems]);
            for (int j = 0; j < nItems; ++j) floats[j] = item.doubleValues[j];
            ps.AddXYZSpectrum(name, std::move(floats), nItems);
        } else if (type == PARAM_TYPE_BLACKBODY) {
            if ((nItems % 2) != 0) {
                Warning(
                    "Excess value given with blackbody parameter \"%s\". "
                    "Ignoring extra one.",
                    item.name.c_str());
                nItems -= nItems % 2;
            }
            std::unique_ptr<Float[]> floats(new Float[nItems]);
            for (int j = 0; j < nItems; ++j) floats[j] = item.doubleValues[j];
            ps.AddBlackbodySpectrum(name, std::move(floats), nItems);
        } else if (type == PARAM_TYPE_SPECTRUM) {
            if (item.stringValues) {
                ps.AddSampledSpectrumFiles(name, item.stringValues, nItems);
            } else {
                if ((nItems % 2) != 0) {
                    Warning(
                        "Non-even number of values given with sampled "
                        "spectrum "
                        "parameter \"%s\". Ignoring extra.",
                        item.name.c_str());
                    nItems -= nItems % 2;
                }
                std::unique_ptr<Float[]> floats(new Float[nItems]);
                for (int j = 0; j < nItems; ++j)
                    floats[j] = item.doubleValues[j];
                ps.AddSampledSpectrum(name, std::move(floats), nItems);
            }
        } else if (type == PARAM_TYPE_STRING) {
            std::unique_ptr<std::string[]> strings(new std::string[nItems]);
            for (int j = 0; j < nItems; ++j)
                strings[j] = std::string(item.stringValues[j]);
            ps.AddString(name, std::move(strings), nItems);
        } else if (type == PARAM_TYPE_TEXTURE) {
            if (nItems == 1) {
                std::string val(*item.stringValues);
                ps.AddTexture(name, val);
            } else
                Error(
                    "Only one string allowed for \"texture\" parameter "
                    "\"%s\"",
                    name.c_str());
        }
    } else
        Warning("Type of parameter \"%s\" is unknown", item.name.c_str());
}

template <typename Next, typename Unget>
ParamSet parseParams(Next nextToken, Unget ungetToken, MemoryArena &arena,
                     SpectrumType spectrumType) {
    ParamSet ps;
    while (true) {
        string_view decl = nextToken(TokenOptional);
        if (decl.empty()) return ps;

        if (!isQuotedString(decl)) {
            ungetToken(decl);
            return ps;
        }

        ParamListItem item;
        item.name = toString(dequoteString(decl));
        size_t nAlloc = 0;

        auto addVal = [&](string_view val) {
            if (isQuotedString(val)) {
                if (item.doubleValues) {
                    Error("mixed string and numeric parameters");
                    exit(1);
                }
                if (item.size == nAlloc) {
                    nAlloc = std::max<size_t>(2 * item.size, 4);
                    const char **newData = arena.Alloc<const char *>(nAlloc);
                    std::copy(item.stringValues, item.stringValues + item.size,
                              newData);
                    item.stringValues = newData;
                }

                val = dequoteString(val);
                char *buf = arena.Alloc<char>(val.size() + 1);
                memcpy(buf, val.data(), val.size());
                buf[val.size()] = '\0';
                item.stringValues[item.size++] = buf;
            } else {
                if (item.stringValues) {
                    Error("mixed string and numeric parameters");
                    exit(1);
                }

                if (item.size == nAlloc) {
                    nAlloc = std::max<size_t>(2 * item.size, 4);
                    double *newData = arena.Alloc<double>(nAlloc);
                    std::copy(item.doubleValues, item.doubleValues + item.size,
                              newData);
                    item.doubleValues = newData;
                }
                item.doubleValues[item.size++] = parseNumber(val);
            }
        };

        string_view val = nextToken(TokenRequired);

        if (val == "[") {
            while (true) {
                val = nextToken(TokenRequired);
                if (val == "]") break;
                addVal(val);
            }
        } else {
            addVal(val);
        }

        AddParam(ps, item, spectrumType);
        arena.Reset();
    }

    return ps;
}

extern int catIndentCount;

// Parsing Global Interface
static void parse(std::unique_ptr<Tokenizer> t) {
    std::vector<std::unique_ptr<Tokenizer>> fileStack;
    fileStack.push_back(std::move(t));
    parserLoc = &fileStack.back()->loc;

    bool ungetTokenSet = false;
    std::string ungetTokenValue;

    // nextToken is a little helper function that handles the file stack,
    // returning the next token from the current file until reaching EOF,
    // at which point it switches to the next file (if any).
    std::function<string_view(int)> nextToken;
    nextToken = [&](int flags) -> string_view {
        if (ungetTokenSet) {
            ungetTokenSet = false;
            return string_view(ungetTokenValue.data(), ungetTokenValue.size());
        }

        if (fileStack.empty()) {
            if (flags & TokenRequired) {
                Error("premature EOF");
                exit(1);
            }
            parserLoc = nullptr;
            return {};
        }

        string_view tok = fileStack.back()->Next();

        if (tok.empty()) {
            // We've reached EOF in the current file. Anything more to parse?
            fileStack.pop_back();
            if (!fileStack.empty()) parserLoc = &fileStack.back()->loc;
            return nextToken(flags);
        } else if (tok[0] == '#') {
            // Swallow comments, unless --cat or --toply was given, in
            // which case they're printed to stdout.
            if (PbrtOptions.cat || PbrtOptions.toPly)
                printf("%*s%s\n", catIndentCount, "", toString(tok).c_str());
            return nextToken(flags);
        } else
            // Regular token; success.
            return tok;
    };

    auto ungetToken = [&](string_view s) {
        CHECK(!ungetTokenSet);
        ungetTokenValue = std::string(s.data(), s.size());
        ungetTokenSet = true;
    };

    MemoryArena arena;

    // Helper function for pbrt API entrypoints that take a single string
    // parameter and a ParamSet (e.g. pbrtShape()).
    auto basicParamListEntrypoint = [&](
        SpectrumType spectrumType,
        std::function<void(const std::string &n, ParamSet p)> apiFunc) {
        string_view token = nextToken(TokenRequired);
        string_view dequoted = dequoteString(token);
        std::string n = toString(dequoted);
        ParamSet params =
            parseParams(nextToken, ungetToken, arena, spectrumType);
        apiFunc(n, std::move(params));
    };

    auto syntaxError = [&](string_view tok) {
        Error("Unexpected token: %s", toString(tok).c_str());
        exit(1);
    };

    while (true) {
        string_view tok = nextToken(TokenOptional);
        if (tok.empty()) break;

        switch (tok[0]) {
        case 'A':
            if (tok == "AttributeBegin")
                pbrtAttributeBegin();
            else if (tok == "AttributeEnd")
                pbrtAttributeEnd();
            else if (tok == "ActiveTransform") {
                string_view a = nextToken(TokenRequired);
                if (a == "All")
                    pbrtActiveTransformAll();
                else if (a == "EndTime")
                    pbrtActiveTransformEndTime();
                else if (a == "StartTime")
                    pbrtActiveTransformStartTime();
                else
                    syntaxError(tok);
            } else if (tok == "AreaLightSource")
                basicParamListEntrypoint(SpectrumType::Illuminant,
                                         pbrtAreaLightSource);
            else if (tok == "Accelerator")
                basicParamListEntrypoint(SpectrumType::Reflectance,
                                         pbrtAccelerator);
            else
                syntaxError(tok);
            break;

        case 'C':
            if (tok == "ConcatTransform") {
                if (nextToken(TokenRequired) != "[") syntaxError(tok);
                Float m[16];
                for (int i = 0; i < 16; ++i)
                    m[i] = parseNumber(nextToken(TokenRequired));
                if (nextToken(TokenRequired) != "]") syntaxError(tok);
                pbrtConcatTransform(m);
            } else if (tok == "CoordinateSystem") {
                string_view n = dequoteString(nextToken(TokenRequired));
                pbrtCoordinateSystem(toString(n));
            } else if (tok == "CoordSysTransform") {
                string_view n = dequoteString(nextToken(TokenRequired));
                pbrtCoordSysTransform(toString(n));
            } else if (tok == "Camera")
                basicParamListEntrypoint(SpectrumType::Reflectance, pbrtCamera);
            else
                syntaxError(tok);
            break;

        case 'F':
            if (tok == "Film")
                basicParamListEntrypoint(SpectrumType::Reflectance, pbrtFilm);
            else
                syntaxError(tok);
            break;

        case 'I':
            if (tok == "Integrator")
                basicParamListEntrypoint(SpectrumType::Reflectance,
                                         pbrtIntegrator);
            else if (tok == "Include") {
                // Switch to the given file.
                std::string filename =
                    toString(dequoteString(nextToken(TokenRequired)));
                if (PbrtOptions.cat || PbrtOptions.toPly)
                    printf("%*sInclude \"%s\"\n", catIndentCount, "", filename.c_str());
                else {
                    filename = AbsolutePath(ResolveFilename(filename));
                    auto tokError = [](const char *msg) { Error("%s", msg); };
                    std::unique_ptr<Tokenizer> tinc =
                        Tokenizer::CreateFromFile(filename, tokError);
                    if (tinc) {
                        fileStack.push_back(std::move(tinc));
                        parserLoc = &fileStack.back()->loc;
                    }
                }
            } else if (tok == "Identity")
                pbrtIdentity();
            else
                syntaxError(tok);
            break;

        case 'L':
            if (tok == "LightSource")
                basicParamListEntrypoint(SpectrumType::Illuminant,
                                         pbrtLightSource);
            else if (tok == "LookAt") {
                Float v[9];
                for (int i = 0; i < 9; ++i)
                    v[i] = parseNumber(nextToken(TokenRequired));
                pbrtLookAt(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7],
                           v[8]);
            } else
                syntaxError(tok);
            break;

        case 'M':
            if (tok == "MakeNamedMaterial")
                basicParamListEntrypoint(SpectrumType::Reflectance,
                                         pbrtMakeNamedMaterial);
            else if (tok == "MakeNamedMedium")
                basicParamListEntrypoint(SpectrumType::Reflectance,
                                         pbrtMakeNamedMedium);
            else if (tok == "Material")
                basicParamListEntrypoint(SpectrumType::Reflectance,
                                         pbrtMaterial);
            else if (tok == "MediumInterface") {
                string_view n = dequoteString(nextToken(TokenRequired));
                std::string names[2];
                names[0] = toString(n);

                // Check for optional second parameter
                string_view second = nextToken(TokenOptional);
                if (!second.empty()) {
                    if (isQuotedString(second))
                        names[1] = toString(dequoteString(second));
                    else {
                        ungetToken(second);
                        names[1] = names[0];
                    }
                } else
                    names[1] = names[0];

                pbrtMediumInterface(names[0], names[1]);
            } else
                syntaxError(tok);
            break;

        case 'N':
            if (tok == "NamedMaterial") {
                string_view n = dequoteString(nextToken(TokenRequired));
                pbrtNamedMaterial(toString(n));
            } else
                syntaxError(tok);
            break;

        case 'O':
            if (tok == "ObjectBegin") {
                string_view n = dequoteString(nextToken(TokenRequired));
                pbrtObjectBegin(toString(n));
            } else if (tok == "ObjectEnd")
                pbrtObjectEnd();
            else if (tok == "ObjectInstance") {
                string_view n = dequoteString(nextToken(TokenRequired));
                pbrtObjectInstance(toString(n));
            } else
                syntaxError(tok);
            break;

        case 'P':
            if (tok == "PixelFilter")
                basicParamListEntrypoint(SpectrumType::Reflectance,
                                         pbrtPixelFilter);
            else
                syntaxError(tok);
            break;

        case 'R':
            if (tok == "ReverseOrientation")
                pbrtReverseOrientation();
            else if (tok == "Rotate") {
                Float v[4];
                for (int i = 0; i < 4; ++i)
                    v[i] = parseNumber(nextToken(TokenRequired));
                pbrtRotate(v[0], v[1], v[2], v[3]);
            } else
                syntaxError(tok);
            break;

        case 'S':
            if (tok == "Shape")
                basicParamListEntrypoint(SpectrumType::Reflectance, pbrtShape);
            else if (tok == "Sampler")
                basicParamListEntrypoint(SpectrumType::Reflectance,
                                         pbrtSampler);
            else if (tok == "Scale") {
                Float v[3];
                for (int i = 0; i < 3; ++i)
                    v[i] = parseNumber(nextToken(TokenRequired));
                pbrtScale(v[0], v[1], v[2]);
            } else
                syntaxError(tok);
            break;

        case 'T':
            if (tok == "TransformBegin")
                pbrtTransformBegin();
            else if (tok == "TransformEnd")
                pbrtTransformEnd();
            else if (tok == "Transform") {
                if (nextToken(TokenRequired) != "[") syntaxError(tok);
                Float m[16];
                for (int i = 0; i < 16; ++i)
                    m[i] = parseNumber(nextToken(TokenRequired));
                if (nextToken(TokenRequired) != "]") syntaxError(tok);
                pbrtTransform(m);
            } else if (tok == "Translate") {
                Float v[3];
                for (int i = 0; i < 3; ++i)
                    v[i] = parseNumber(nextToken(TokenRequired));
                pbrtTranslate(v[0], v[1], v[2]);
            } else if (tok == "TransformTimes") {
                Float v[2];
                for (int i = 0; i < 2; ++i)
                    v[i] = parseNumber(nextToken(TokenRequired));
                pbrtTransformTimes(v[0], v[1]);
            } else if (tok == "Texture") {
                string_view n = dequoteString(nextToken(TokenRequired));
                std::string name = toString(n);
                n = dequoteString(nextToken(TokenRequired));
                std::string type = toString(n);

                basicParamListEntrypoint(
                    SpectrumType::Reflectance,
                    [&](const std::string &texName, const ParamSet &params) {
                        pbrtTexture(name, type, texName, params);
                    });
            } else
                syntaxError(tok);
            break;

        case 'W':
            if (tok == "WorldBegin")
                pbrtWorldBegin();
            else if (tok == "WorldEnd")
                pbrtWorldEnd();
            else
                syntaxError(tok);
            break;

        default:
            syntaxError(tok);
        }
    }
}

void pbrtParseFile(std::string filename) {
    if (filename != "-") SetSearchDirectory(DirectoryContaining(filename));

    auto tokError = [](const char *msg) { Error("%s", msg); exit(1); };
    std::unique_ptr<Tokenizer> t =
        Tokenizer::CreateFromFile(filename, tokError);
    if (!t) return;
    parse(std::move(t));
}

void pbrtParseString(std::string str) {
    auto tokError = [](const char *msg) { Error("%s", msg); exit(1); };
    std::unique_ptr<Tokenizer> t =
        Tokenizer::CreateFromString(std::move(str), tokError);
    if (!t) return;
    parse(std::move(t));
}

}  // namespace pbrt
