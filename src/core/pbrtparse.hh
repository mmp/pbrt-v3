/* A Bison parser, made by GNU Bison 2.3.  */

/* Skeleton interface for Bison's Yacc-like parsers in C

   Copyright (C) 1984, 1989, 1990, 2000, 2001, 2002, 2003, 2004, 2005, 2006
   Free Software Foundation, Inc.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.  */

/* As a special exception, you may create a larger work that contains
   part or all of the Bison parser skeleton and distribute that work
   under terms of your choice, so long as that work isn't itself a
   parser generator using the skeleton or a modified version thereof
   as a parser skeleton.  Alternatively, if you modify or redistribute
   the parser skeleton itself, you may (at your option) remove this
   special exception, which will cause the skeleton and the resulting
   Bison output files to be licensed under the GNU General Public
   License without this special exception.

   This special exception was added by the Free Software Foundation in
   version 2.2 of Bison.  */

/* Tokens.  */
#ifndef YYTOKENTYPE
# define YYTOKENTYPE
   /* Put the tokens into the symbol table, so that GDB and other debuggers
      know about them.  */
   enum yytokentype {
     STRING = 258,
     ID = 259,
     NUM = 260,
     LBRACK = 261,
     RBRACK = 262,
     ACCELERATOR = 263,
     ACTIVETRANSFORM = 264,
     ALL = 265,
     AREALIGHTSOURCE = 266,
     ATTRIBUTEBEGIN = 267,
     ATTRIBUTEEND = 268,
     CAMERA = 269,
     CONCATTRANSFORM = 270,
     COORDINATESYSTEM = 271,
     COORDSYSTRANSFORM = 272,
     ENDTIME = 273,
     FILM = 274,
     IDENTITY = 275,
     INCLUDE = 276,
     LIGHTSOURCE = 277,
     LOOKAT = 278,
     MAKENAMEDMATERIAL = 279,
     MAKENAMEDMEDIUM = 280,
     MATERIAL = 281,
     MEDIUMINTERFACE = 282,
     NAMEDMATERIAL = 283,
     OBJECTBEGIN = 284,
     OBJECTEND = 285,
     OBJECTINSTANCE = 286,
     PIXELFILTER = 287,
     REVERSEORIENTATION = 288,
     ROTATE = 289,
     SAMPLER = 290,
     SCALE = 291,
     SHAPE = 292,
     STARTTIME = 293,
     INTEGRATOR = 294,
     TEXTURE = 295,
     TRANSFORMBEGIN = 296,
     TRANSFORMEND = 297,
     TRANSFORMTIMES = 298,
     TRANSFORM = 299,
     TRANSLATE = 300,
     WORLDBEGIN = 301,
     WORLDEND = 302,
     HIGH_PRECEDENCE = 303
   };
#endif
/* Tokens.  */
#define STRING 258
#define ID 259
#define NUM 260
#define LBRACK 261
#define RBRACK 262
#define ACCELERATOR 263
#define ACTIVETRANSFORM 264
#define ALL 265
#define AREALIGHTSOURCE 266
#define ATTRIBUTEBEGIN 267
#define ATTRIBUTEEND 268
#define CAMERA 269
#define CONCATTRANSFORM 270
#define COORDINATESYSTEM 271
#define COORDSYSTRANSFORM 272
#define ENDTIME 273
#define FILM 274
#define IDENTITY 275
#define INCLUDE 276
#define LIGHTSOURCE 277
#define LOOKAT 278
#define MAKENAMEDMATERIAL 279
#define MAKENAMEDMEDIUM 280
#define MATERIAL 281
#define MEDIUMINTERFACE 282
#define NAMEDMATERIAL 283
#define OBJECTBEGIN 284
#define OBJECTEND 285
#define OBJECTINSTANCE 286
#define PIXELFILTER 287
#define REVERSEORIENTATION 288
#define ROTATE 289
#define SAMPLER 290
#define SCALE 291
#define SHAPE 292
#define STARTTIME 293
#define INTEGRATOR 294
#define TEXTURE 295
#define TRANSFORMBEGIN 296
#define TRANSFORMEND 297
#define TRANSFORMTIMES 298
#define TRANSFORM 299
#define TRANSLATE 300
#define WORLDBEGIN 301
#define WORLDEND 302
#define HIGH_PRECEDENCE 303




#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
typedef union YYSTYPE
#line 159 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
{
char string[1024];
double num;
ParamArray *ribarray;
}
/* Line 1529 of yacc.c.  */
#line 151 "/Users/mmp/build/pbrt-v3/pbrtparse.hpp"
	YYSTYPE;
# define yystype YYSTYPE /* obsolescent; will be withdrawn */
# define YYSTYPE_IS_DECLARED 1
# define YYSTYPE_IS_TRIVIAL 1
#endif

extern YYSTYPE yylval;

