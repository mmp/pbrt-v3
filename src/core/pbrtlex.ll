
/*
    pbrt source code is Copyright(c) 1998-2015
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

/* state used for include file stuff */
%{

#define YY_MAIN 0
#define YY_NO_INPUT 1
#define YY_NEVER_INTERACTIVE 1

#include "pbrt.h"
#include "api.h"
#include "fileutil.h"

struct ParamArray;

#if defined(PBRT_IS_MSVC)
#pragma warning(disable:4244)
#pragma warning(disable:4065)
#pragma warning(disable:4018)
#pragma warning(disable:4996)
#endif  // PBRT_IS_MSVC
#include "pbrtparse.hh"

struct IncludeInfo {
    std::string filename;
    YY_BUFFER_STATE bufState;
    int lineNum;
};


std::vector<IncludeInfo> includeStack;

extern int line_num;
int str_pos;

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


%}
%option nounput
WHITESPACE [ \t\r]+
NUMBER [-+]?([0-9]+|(([0-9]+\.[0-9]*)|(\.[0-9]+)))([eE][-+]?[0-9]+)?
IDENT [a-zA-Z_][a-zA-Z_0-9]*
%x STR COMMENT INCL INCL_FILE
%%
"#" { BEGIN COMMENT; }
<COMMENT>. /* eat it up */
<COMMENT>\n { line_num++; BEGIN INITIAL; }
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
\n { line_num++; }
{NUMBER} {
    yylval.num = (Float) atof(yytext);
    return NUM;
}


{IDENT} {
    strlcpy(yylval.string, yytext, sizeof(yytext));
    return ID;
}


"[" { return LBRACK; }
"]" { return RBRACK; }
\" { BEGIN STR; str_pos = 0; yylval.string[0] = '\0'; }
<STR>\\n {add_string_char('\n');}
<STR>\\t {add_string_char('\t');}
<STR>\\r {add_string_char('\r');}
<STR>\\b {add_string_char('\b');}
<STR>\\f {add_string_char('\f');}
<STR>\\\" {add_string_char('\"');}
<STR>\\\\ {add_string_char('\\');}
<STR>\\[0-9]{3} {
  int val = atoi(yytext+1);
  while (val > 256)
    val -= 256;
  add_string_char(val);
}


<STR>\\\n {line_num++;}
<STR>\\. { add_string_char(yytext[1]);}
<STR>\" {BEGIN INITIAL; return STRING;}
<STR>. {add_string_char(yytext[0]);}
<STR>\n {Error("Unterminated string!");}

. { Error( "Illegal character: %c (0x%x)", yytext[0], int(yytext[0])); }
%%
int yywrap() {
    if (includeStack.size() == 0) return 1;
    include_pop();
    return 0;
}



