
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


// core/fileutil.cpp*
#include "fileutil.h"
#include <cstdlib>
#include <climits>
#ifndef PBRT_IS_WINDOWS
#include <libgen.h>
#endif

namespace pbrt {

static std::string searchDirectory;

#ifdef PBRT_IS_WINDOWS
bool IsAbsolutePath(const std::string &filename) {
    if (filename.size() == 0) return false;
    return (filename[0] == '\\' || filename[0] == '/' ||
            filename.find(':') != std::string::npos);
}

std::string AbsolutePath(const std::string &filename) {
    char full[_MAX_PATH];
    if (_fullpath(full, filename.c_str(), _MAX_PATH))
        return std::string(full);
    else
        return filename;
}

std::string ResolveFilename(const std::string &filename) {
    if (searchDirectory.size() == 0 || filename.size() == 0)
        return filename;
    else if (IsAbsolutePath(filename))
        return filename;

    char searchDirectoryEnd = searchDirectory[searchDirectory.size() - 1];
    if (searchDirectoryEnd == '\\' || searchDirectoryEnd == '/')
        return searchDirectory + filename;
    else
        return searchDirectory + "\\" + filename;
}

std::string DirectoryContaining(const std::string &filename) {
    // This code isn't tested but I believe it should work. Might need to add
    // some const_casts to make it compile though.
    char drive[_MAX_DRIVE];
    char dir[_MAX_DIR];
    char ext[_MAX_EXT];

    errno_t err = _splitpath_s(filename.c_str(), drive, _MAX_DRIVE, dir,
                               _MAX_DIR, nullptr, 0, ext, _MAX_EXT);
    if (err == 0) {
        char fullDir[_MAX_PATH];
        err = _makepath_s(fullDir, _MAX_PATH, drive, dir, nullptr, nullptr);
        if (err == 0) return std::string(fullDir);
    }
    return filename;
}

#else

bool IsAbsolutePath(const std::string &filename) {
    return (filename.size() > 0) && filename[0] == '/';
}

std::string AbsolutePath(const std::string &filename) {
    char full[PATH_MAX];
    if (realpath(filename.c_str(), full))
        return std::string(full);
    else
        return filename;
}

std::string ResolveFilename(const std::string &filename) {
    if (searchDirectory.size() == 0 || filename.size() == 0)
        return filename;
    else if (IsAbsolutePath(filename))
        return filename;
    else if (searchDirectory[searchDirectory.size() - 1] == '/')
        return searchDirectory + filename;
    else
        return searchDirectory + "/" + filename;
}

std::string DirectoryContaining(const std::string &filename) {
    char *t = strdup(filename.c_str());
    std::string result = dirname(t);
    free(t);
    return result;
}

#endif

void SetSearchDirectory(const std::string &dirname) {
    searchDirectory = dirname;
}

}  // namespace pbrt
