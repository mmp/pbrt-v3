
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
#include "fileutil.h"

// Parsing Global Interface
bool ParseFile(const std::string &filename) {
    extern FILE *yyin;
    extern int yyparse(void);
    extern std::string current_file;
    extern int line_num;
    extern int yydebug;

    LOG(INFO) << "Starting to parse input file " << filename;

    if (getenv("PBRT_YYDEBUG") != nullptr) yydebug = 1;

    if (filename == "-")
        yyin = stdin;
    else {
        yyin = fopen(filename.c_str(), "r");
        SetSearchDirectory(DirectoryContaining(filename));
    }
    if (yyin != nullptr) {
        current_file = filename;
        if (yyin == stdin) current_file = "<standard input>";
        line_num = 1;
        yyparse();
        if (yyin != stdin) fclose(yyin);
    }
    current_file = "";
    line_num = 0;
    LOG(INFO) << "Done parsing input file " << filename;
    return (yyin != nullptr);
}
