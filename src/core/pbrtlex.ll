
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

%option nounistd

%{

#include "pbrt.h"
#include "api.h"
#include "fileutil.h"

#if defined(PBRT_IS_MSVC)
#include <io.h>
#pragma warning(disable:4244)
#pragma warning(disable:4065)
#pragma warning(disable:4018)
#pragma warning(disable:4996)
int isatty(int fd) { return _isatty(fd); }
#else
#include <unistd.h>
#endif  // PBRT_IS_MSVC

namespace pbrt {
struct ParamArray;
}

#include "pbrtparse.h"

namespace pbrt {

struct IncludeInfo {
    std::string filename;
    YY_BUFFER_STATE bufState;
    int lineNum;
};

std::vector<IncludeInfo> includeStack;

extern int line_num;
int str_pos;
extern int catIndentCount;

void add_string_char(char c) {
    yylval.string[str_pos++] = c;
    yylval.string[str_pos] = '\0';
}


void include_push(char *filename) {
    if (includeStack.size() > 32) {
        Error("Only 32 levels of nested Include allowed in scene files.");
        exit(1);
    }

    std::string new_file = AbsolutePath(ResolveFilename(filename));

    FILE *f = fopen(new_file.c_str(), "r");
    if (!f)
        Error("Unable to open included scene file \"%s\"", new_file.c_str());
    else {
        extern std::string current_file;
        IncludeInfo ii;
        ii.filename = current_file;
        ii.bufState = YY_CURRENT_BUFFER;
        ii.lineNum = line_num;
        includeStack.push_back(ii);

        yyin = f;
        current_file = new_file;
        line_num = 1;

        yy_switch_to_buffer(yy_create_buffer(yyin, YY_BUF_SIZE));
    }
}



void include_pop() {
    extern int line_num;
    extern std::string current_file;
    fclose(yyin);
    yy_delete_buffer(YY_CURRENT_BUFFER);
    yy_switch_to_buffer(includeStack.back().bufState);
    current_file = includeStack.back().filename;
    line_num = includeStack.back().lineNum;
    includeStack.pop_back();
}

}  // namespace pbrt

%}
%option nounput
WHITESPACE [ \t\r]+
NUMBER [-+]?([0-9]+|(([0-9]+\.[0-9]*)|(\.[0-9]+)))([eE][-+]?[0-9]+)?
IDENT [a-zA-Z_][a-zA-Z_0-9]*
%x STR COMMENT INCL INCL_FILE
%%
"#" { BEGIN COMMENT; if (pbrt::PbrtOptions.cat || pbrt::PbrtOptions.toPly) printf("%*s#", pbrt::catIndentCount, ""); }
<COMMENT>. { /* eat it up */ if (pbrt::PbrtOptions.cat || pbrt::PbrtOptions.toPly) putchar(yytext[0]); }
<COMMENT>\n { pbrt::line_num++; if (pbrt::PbrtOptions.cat || pbrt::PbrtOptions.toPly) putchar('\n'); BEGIN INITIAL; }
Accelerator             { return ACCELERATOR; }
ActiveTransform         { return ACTIVETRANSFORM; }
All                     { return ALL; }
AreaLightSource         { return AREALIGHTSOURCE; }
AttributeBegin          { return ATTRIBUTEBEGIN; }
AttributeEnd            { return ATTRIBUTEEND; }
Camera                  { return CAMERA; }
ConcatTransform         { return CONCATTRANSFORM; }
CoordinateSystem        { return COORDINATESYSTEM; }
CoordSysTransform       { return COORDSYSTRANSFORM; }
EndTime                 { return ENDTIME; }
Film                    { return FILM; }
Identity                { return IDENTITY; }
Include                 { return INCLUDE; }
LightSource             { return LIGHTSOURCE; }
LookAt                  { return LOOKAT; }
MakeNamedMedium         { return MAKENAMEDMEDIUM; }
MakeNamedMaterial       { return MAKENAMEDMATERIAL; }
Material                { return MATERIAL; }
MediumInterface         { return MEDIUMINTERFACE; }
NamedMaterial           { return NAMEDMATERIAL; }
ObjectBegin             { return OBJECTBEGIN; }
ObjectEnd               { return OBJECTEND; }
ObjectInstance          { return OBJECTINSTANCE; }
PixelFilter             { return PIXELFILTER; }
ReverseOrientation      { return REVERSEORIENTATION; }
Rotate                  { return ROTATE; }
Sampler                 { return SAMPLER; }
Scale                   { return SCALE; }
Shape                   { return SHAPE; }
StartTime               { return STARTTIME; }
Integrator              { return INTEGRATOR; }
Texture                 { return TEXTURE; }
TransformBegin          { return TRANSFORMBEGIN; }
TransformEnd            { return TRANSFORMEND; }
TransformTimes          { return TRANSFORMTIMES; }
Transform               { return TRANSFORM; }
Translate               { return TRANSLATE; }
WorldBegin              { return WORLDBEGIN; }
WorldEnd                { return WORLDEND; }
{WHITESPACE} /* do nothing */
\n { pbrt::line_num++; }
{NUMBER} {
    yylval.num = atof(yytext);
    return NUM;
}


{IDENT} {
    yylval.string[0] = '\0';
    strncat(yylval.string, yytext, sizeof(yylval.string) - 1);
    return ID;
}


"[" { return LBRACK; }
"]" { return RBRACK; }
\" { BEGIN STR; pbrt::str_pos = 0; yylval.string[0] = '\0'; }
<STR>\\n {pbrt::add_string_char('\n');}
<STR>\\t {pbrt::add_string_char('\t');}
<STR>\\r {pbrt::add_string_char('\r');}
<STR>\\b {pbrt::add_string_char('\b');}
<STR>\\f {pbrt::add_string_char('\f');}
<STR>\\\" {pbrt::add_string_char('\"');}
<STR>\\\\ {pbrt::add_string_char('\\');}
<STR>\\[0-9]{3} {
  int val = atoi(yytext+1);
  while (val > 256)
    val -= 256;
  pbrt::add_string_char(val);
}


<STR>\\\n {pbrt::line_num++;}
<STR>\\. { pbrt::add_string_char(yytext[1]);}
<STR>\" {BEGIN INITIAL; return STRING;}
<STR>. {pbrt::add_string_char(yytext[0]);}
<STR>\n {pbrt::Error("Unterminated string!");}

. { pbrt::Error("Illegal character: %c (0x%x)", yytext[0], int(yytext[0])); }
%%
int yywrap() {
    if (pbrt::includeStack.size() == 0) return 1;
    pbrt::include_pop();
    return 0;
}



