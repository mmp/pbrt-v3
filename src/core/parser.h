
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

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CORE_PARSER_H
#define PBRT_CORE_PARSER_H

// core/parser.h*
#include "pbrt.h"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace pbrt {

// Loc represents a position in a file being parsed.
struct Loc {
    Loc() = default;
    Loc(const std::string &filename) : filename(filename) {}

    std::string filename;
    int line = 1, column = 0;
};

// If not nullptr, stores the current file location of the parser.
extern Loc *parserLoc;

// Reimplement enough of absl/std::string_view as needed for the below
// (Bringing on the abseil dependency at this point just for this seems
// excessive.)
class string_view {
  public:
    string_view(const char *start, size_t size) : ptr(start), length(size) {}
    string_view() : ptr(nullptr), length(0) {}

    const char *data() const { return ptr; }
    size_t size() const { return length; }
    bool empty() const { return length == 0; }

    char operator[](int index) const { return ptr[index]; }
    char back() const { return ptr[length - 1]; }

    const char *begin() const { return ptr; }
    const char *end() const { return ptr + length; }

    bool operator==(const char *str) const {
        int index;
        for (index = 0; *str; ++index, ++str) {
            if (index >= length) return false;
            if (*str != ptr[index]) return false;
        }
        return index == length;
    }
    bool operator!=(const char *str) const { return !(*this == str); }

    void remove_prefix(int n) {
        ptr += n;
        length -= n;
    }
    void remove_suffix(int n) { length -= n; }

  private:
    const char *ptr;
    size_t length;
};

// Tokenizer converts a single pbrt scene file into a series of tokens.
class Tokenizer {
  public:
    static std::unique_ptr<Tokenizer> CreateFromFile(
        const std::string &filename,
        std::function<void(const char *)> errorCallback);
    static std::unique_ptr<Tokenizer> CreateFromString(
        std::string str, std::function<void(const char *)> errorCallback);

    ~Tokenizer();

    // Returns an empty string_view at EOF. Note that the returned
    // string_view is not guaranteed to be valid after next call to Next().
    string_view Next();

    Loc loc;

  private:
    Tokenizer(std::string str, std::function<void(const char *)> errorCallback);
#if defined(PBRT_HAVE_MMAP) || defined(PBRT_IS_WINDOWS)
    Tokenizer(void *ptr, size_t len, std::string filename,
              std::function<void(const char *)> errorCallback);
#endif

    int getChar() {
        if (pos == end) return EOF;
        int ch = *pos++;
        if (ch == '\n') {
            ++loc.line;
            loc.column = 0;
        } else
            ++loc.column;
        return ch;
    }
    void ungetChar() {
        --pos;
        if (*pos == '\n')
            // Don't worry about the column; we'll be going to the start of
            // the next line again shortly...
            --loc.line;
    }

    // This function is called if there is an error during lexing.
    std::function<void(const char *)> errorCallback;

#if defined(PBRT_HAVE_MMAP) || defined(PBRT_IS_WINDOWS)
    // Scene files on disk are mapped into memory for lexing.  We need to
    // hold on to the starting pointer and total length so they can be
    // unmapped in the destructor.
    void *unmapPtr = nullptr;
    size_t unmapLength = 0;
#endif

    // If the input is stdin, then we copy everything until EOF into this
    // string and then start lexing.  This is a little wasteful (versus
    // tokenizing directly from stdin), but makes the implementation
    // simpler.
    std::string contents;

    // Pointers to the current position inthe file and one past the end of
    // the file.
    const char *pos, *end;

    // If there are escaped characters in the string, we can't just return
    // a string_view into the mapped file. In that case, we handle the
    // escaped characters and return a string_view to sEscaped.  (And
    // thence, string_views from previous calls to Next() must be invalid
    // after a subsequent call, since we may reuse sEscaped.)
    std::string sEscaped;
};

}  // namespace pbrt

#endif  // PBRT_CORE_PARSER_H
