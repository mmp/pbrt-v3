/* A Bison parser, made by GNU Bison 2.3.  */

/* Skeleton implementation for Bison's Yacc-like parsers in C

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

/* C LALR(1) parser skeleton written by Richard Stallman, by
   simplifying the original so-called "semantic" parser.  */

/* All symbols defined below should begin with yy or YY, to avoid
   infringing on user name space.  This should be done even for local
   variables, as they might otherwise be expanded by user macros.
   There are some unavoidable exceptions within include files to
   define necessary library symbols; they are noted "INFRINGES ON
   USER NAME SPACE" below.  */

/* Identify Bison output.  */
#define YYBISON 1

/* Bison version.  */
#define YYBISON_VERSION "2.3"

/* Skeleton name.  */
#define YYSKELETON_NAME "yacc.c"

/* Pure parsers.  */
#define YYPURE 0

/* Using locations.  */
#define YYLSP_NEEDED 0



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




/* Copy the first part of user declarations.  */
#line 33 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"

#include "api.h"
#include "pbrt.h"
#include "paramset.h"
#include <stdarg.h>

#ifdef PBRT_IS_MSVC
#pragma warning(disable:4065)
#pragma warning(disable:4996)
#pragma warning(disable:4018)
#endif // PBRT_IS_MSVC

extern int yylex();
extern void include_push(char *filename);
int line_num = 0;
std::string current_file;

#define YYMAXDEPTH 100000000

void yyerror(const char *str) {
    Error("Parsing error: %s", str);
    exit(1);
}



struct ParamArray {
    ParamArray() {
        isString = false;
        element_size = allocated = nelems = 0;
        array = nullptr;
    }
    bool isString;
    int element_size;
    int allocated;
    int nelems;
    void *array;
};



struct ParamListItem {
    ParamListItem(const char *t, ParamArray *array) {
        arg = array->array;
        name = t;
        size = array->nelems;
        isString = array->isString;
        array->allocated = 0;
        array->nelems = 0;
        array->array = nullptr;
    }
    const char *name;
    void *arg;
    int size;
    bool isString;
};



static std::vector<ParamListItem> cur_paramlist;

static ParamArray *cur_array = nullptr;

static void AddArrayElement(void *elem) {
    if (cur_array->nelems >= cur_array->allocated) {
        cur_array->allocated = 2*cur_array->allocated + 1;
        cur_array->array = realloc(cur_array->array,
            cur_array->allocated*cur_array->element_size);
    }
    char *next = ((char *)cur_array->array) + cur_array->nelems * cur_array->element_size;
    CHECK(cur_array->element_size == 4 || cur_array->element_size == 8);
    if (cur_array->element_size == 4)
        *((uint32_t *)next) = *((uint32_t *)elem);
    else
        *((uint64_t *)next) = *((uint64_t *)elem);
    cur_array->nelems++;
}



static void ArrayFree(ParamArray *ra) {
    if (ra->isString && ra->array)
        for (int i = 0; i < ra->nelems; ++i) free(((char **)ra->array)[i]);
    free(ra->array);
    delete ra;
}



static void FreeArgs() {
    for (size_t i = 0; i < cur_paramlist.size(); ++i)
        free((char *)cur_paramlist[i].arg);
    cur_paramlist.erase(cur_paramlist.begin(), cur_paramlist.end());
}



static bool VerifyArrayLength(ParamArray *arr, int required,
    const char *command) {
    if (arr->nelems != required) {
        Error("\"%s\" requires a %d element array! (%d found)",
                    command, required, arr->nelems);
        return false;
    }
    return true;
}


enum { PARAM_TYPE_INT, PARAM_TYPE_BOOL, PARAM_TYPE_FLOAT,
    PARAM_TYPE_POINT2, PARAM_TYPE_VECTOR2, PARAM_TYPE_POINT3,
    PARAM_TYPE_VECTOR3, PARAM_TYPE_NORMAL, PARAM_TYPE_RGB, PARAM_TYPE_XYZ,
    PARAM_TYPE_BLACKBODY, PARAM_TYPE_SPECTRUM,
    PARAM_TYPE_STRING, PARAM_TYPE_TEXTURE };
static const char *paramTypeToName(int type);
static void InitParamSet(ParamSet &ps, SpectrumType);
static bool lookupType(const char *name, int *type, std::string &sname);
#define YYPRINT(file, type, value)  { \
    if ((type) == ID || (type) == STRING) \
        fprintf ((file), " %s", (value).string); \
    else if ((type) == NUM) \
        fprintf ((file), " %f", (value).num); \
}




/* Enabling traces.  */
#ifndef YYDEBUG
# define YYDEBUG 1
#endif

/* Enabling verbose error messages.  */
#ifdef YYERROR_VERBOSE
# undef YYERROR_VERBOSE
# define YYERROR_VERBOSE 1
#else
# define YYERROR_VERBOSE 0
#endif

/* Enabling the token table.  */
#ifndef YYTOKEN_TABLE
# define YYTOKEN_TABLE 0
#endif

#if ! defined YYSTYPE && ! defined YYSTYPE_IS_DECLARED
typedef union YYSTYPE
#line 159 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
{
char string[1024];
double num;
ParamArray *ribarray;
}
/* Line 193 of yacc.c.  */
#line 324 "/Users/mmp/build/pbrt-v3/pbrtparse.cpp"
	YYSTYPE;
# define yystype YYSTYPE /* obsolescent; will be withdrawn */
# define YYSTYPE_IS_DECLARED 1
# define YYSTYPE_IS_TRIVIAL 1
#endif



/* Copy the second part of user declarations.  */


/* Line 216 of yacc.c.  */
#line 337 "/Users/mmp/build/pbrt-v3/pbrtparse.cpp"

#ifdef short
# undef short
#endif

#ifdef YYTYPE_UINT8
typedef YYTYPE_UINT8 yytype_uint8;
#else
typedef unsigned char yytype_uint8;
#endif

#ifdef YYTYPE_INT8
typedef YYTYPE_INT8 yytype_int8;
#elif (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
typedef signed char yytype_int8;
#else
typedef short int yytype_int8;
#endif

#ifdef YYTYPE_UINT16
typedef YYTYPE_UINT16 yytype_uint16;
#else
typedef unsigned short int yytype_uint16;
#endif

#ifdef YYTYPE_INT16
typedef YYTYPE_INT16 yytype_int16;
#else
typedef short int yytype_int16;
#endif

#ifndef YYSIZE_T
# ifdef __SIZE_TYPE__
#  define YYSIZE_T __SIZE_TYPE__
# elif defined size_t
#  define YYSIZE_T size_t
# elif ! defined YYSIZE_T && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
#  include <stddef.h> /* INFRINGES ON USER NAME SPACE */
#  define YYSIZE_T size_t
# else
#  define YYSIZE_T unsigned int
# endif
#endif

#define YYSIZE_MAXIMUM ((YYSIZE_T) -1)

#ifndef YY_
# if defined YYENABLE_NLS && YYENABLE_NLS
#  if ENABLE_NLS
#   include <libintl.h> /* INFRINGES ON USER NAME SPACE */
#   define YY_(msgid) dgettext ("bison-runtime", msgid)
#  endif
# endif
# ifndef YY_
#  define YY_(msgid) msgid
# endif
#endif

/* Suppress unused-variable warnings by "using" E.  */
#if ! defined lint || defined __GNUC__
# define YYUSE(e) ((void) (e))
#else
# define YYUSE(e) /* empty */
#endif

/* Identity function, used to suppress warnings about constant conditions.  */
#ifndef lint
# define YYID(n) (n)
#else
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static int
YYID (int i)
#else
static int
YYID (i)
    int i;
#endif
{
  return i;
}
#endif

#if ! defined yyoverflow || YYERROR_VERBOSE

/* The parser invokes alloca or malloc; define the necessary symbols.  */

# ifdef YYSTACK_USE_ALLOCA
#  if YYSTACK_USE_ALLOCA
#   ifdef __GNUC__
#    define YYSTACK_ALLOC __builtin_alloca
#   elif defined __BUILTIN_VA_ARG_INCR
#    include <alloca.h> /* INFRINGES ON USER NAME SPACE */
#   elif defined _AIX
#    define YYSTACK_ALLOC __alloca
#   elif defined _MSC_VER
#    include <malloc.h> /* INFRINGES ON USER NAME SPACE */
#    define alloca _alloca
#   else
#    define YYSTACK_ALLOC alloca
#    if ! defined _ALLOCA_H && ! defined _STDLIB_H && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
#     include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#     ifndef _STDLIB_H
#      define _STDLIB_H 1
#     endif
#    endif
#   endif
#  endif
# endif

# ifdef YYSTACK_ALLOC
   /* Pacify GCC's `empty if-body' warning.  */
#  define YYSTACK_FREE(Ptr) do { /* empty */; } while (YYID (0))
#  ifndef YYSTACK_ALLOC_MAXIMUM
    /* The OS might guarantee only one guard page at the bottom of the stack,
       and a page size can be as small as 4096 bytes.  So we cannot safely
       invoke alloca (N) if N exceeds 4096.  Use a slightly smaller number
       to allow for a few compiler-allocated temporary stack slots.  */
#   define YYSTACK_ALLOC_MAXIMUM 4032 /* reasonable circa 2006 */
#  endif
# else
#  define YYSTACK_ALLOC YYMALLOC
#  define YYSTACK_FREE YYFREE
#  ifndef YYSTACK_ALLOC_MAXIMUM
#   define YYSTACK_ALLOC_MAXIMUM YYSIZE_MAXIMUM
#  endif
#  if (defined __cplusplus && ! defined _STDLIB_H \
       && ! ((defined YYMALLOC || defined malloc) \
	     && (defined YYFREE || defined free)))
#   include <stdlib.h> /* INFRINGES ON USER NAME SPACE */
#   ifndef _STDLIB_H
#    define _STDLIB_H 1
#   endif
#  endif
#  ifndef YYMALLOC
#   define YYMALLOC malloc
#   if ! defined malloc && ! defined _STDLIB_H && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
void *malloc (YYSIZE_T); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
#  ifndef YYFREE
#   define YYFREE free
#   if ! defined free && ! defined _STDLIB_H && (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
void free (void *); /* INFRINGES ON USER NAME SPACE */
#   endif
#  endif
# endif
#endif /* ! defined yyoverflow || YYERROR_VERBOSE */


#if (! defined yyoverflow \
     && (! defined __cplusplus \
	 || (defined YYSTYPE_IS_TRIVIAL && YYSTYPE_IS_TRIVIAL)))

/* A type that is properly aligned for any stack member.  */
union yyalloc
{
  yytype_int16 yyss;
  YYSTYPE yyvs;
  };

/* The size of the maximum gap between one aligned stack and the next.  */
# define YYSTACK_GAP_MAXIMUM (sizeof (union yyalloc) - 1)

/* The size of an array large to enough to hold all stacks, each with
   N elements.  */
# define YYSTACK_BYTES(N) \
     ((N) * (sizeof (yytype_int16) + sizeof (YYSTYPE)) \
      + YYSTACK_GAP_MAXIMUM)

/* Copy COUNT objects from FROM to TO.  The source and destination do
   not overlap.  */
# ifndef YYCOPY
#  if defined __GNUC__ && 1 < __GNUC__
#   define YYCOPY(To, From, Count) \
      __builtin_memcpy (To, From, (Count) * sizeof (*(From)))
#  else
#   define YYCOPY(To, From, Count)		\
      do					\
	{					\
	  YYSIZE_T yyi;				\
	  for (yyi = 0; yyi < (Count); yyi++)	\
	    (To)[yyi] = (From)[yyi];		\
	}					\
      while (YYID (0))
#  endif
# endif

/* Relocate STACK from its old location to the new one.  The
   local variables YYSIZE and YYSTACKSIZE give the old and new number of
   elements in the stack, and YYPTR gives the new location of the
   stack.  Advance YYPTR to a properly aligned location for the next
   stack.  */
# define YYSTACK_RELOCATE(Stack)					\
    do									\
      {									\
	YYSIZE_T yynewbytes;						\
	YYCOPY (&yyptr->Stack, Stack, yysize);				\
	Stack = &yyptr->Stack;						\
	yynewbytes = yystacksize * sizeof (*Stack) + YYSTACK_GAP_MAXIMUM; \
	yyptr += yynewbytes / sizeof (*yyptr);				\
      }									\
    while (YYID (0))

#endif

/* YYFINAL -- State number of the termination state.  */
#define YYFINAL  73
/* YYLAST -- Last index in YYTABLE.  */
#define YYLAST   115

/* YYNTOKENS -- Number of terminals.  */
#define YYNTOKENS  49
/* YYNNTS -- Number of nonterminals.  */
#define YYNNTS  20
/* YYNRULES -- Number of rules.  */
#define YYNRULES  66
/* YYNRULES -- Number of states.  */
#define YYNSTATES  133

/* YYTRANSLATE(YYLEX) -- Bison symbol number corresponding to YYLEX.  */
#define YYUNDEFTOK  2
#define YYMAXUTOK   303

#define YYTRANSLATE(YYX)						\
  ((unsigned int) (YYX) <= YYMAXUTOK ? yytranslate[YYX] : YYUNDEFTOK)

/* YYTRANSLATE[YYLEX] -- Bison symbol number corresponding to YYLEX.  */
static const yytype_uint8 yytranslate[] =
{
       0,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     2,     2,     2,     2,
       2,     2,     2,     2,     2,     2,     1,     2,     3,     4,
       5,     6,     7,     8,     9,    10,    11,    12,    13,    14,
      15,    16,    17,    18,    19,    20,    21,    22,    23,    24,
      25,    26,    27,    28,    29,    30,    31,    32,    33,    34,
      35,    36,    37,    38,    39,    40,    41,    42,    43,    44,
      45,    46,    47,    48
};

#if YYDEBUG
/* YYPRHS[YYN] -- Index of the first RHS symbol of rule number YYN in
   YYRHS.  */
static const yytype_uint8 yyprhs[] =
{
       0,     0,     3,     5,     6,     7,     8,    10,    12,    17,
      19,    22,    25,    27,    30,    35,    37,    40,    43,    45,
      48,    51,    52,    55,    56,    59,    62,    64,    68,    71,
      74,    77,    81,    83,    85,    89,    92,    95,    98,   102,
     104,   107,   111,   122,   126,   130,   134,   137,   141,   144,
     147,   149,   152,   156,   158,   164,   168,   173,   177,   181,
     187,   189,   191,   195,   198,   203,   205
};

/* YYRHS -- A `-1'-separated list of the rules' RHS.  */
static const yytype_int8 yyrhs[] =
{
      50,     0,    -1,    67,    -1,    -1,    -1,    -1,    55,    -1,
      59,    -1,    51,     6,    57,     7,    -1,    56,    -1,    51,
      58,    -1,    57,    58,    -1,    58,    -1,    52,     3,    -1,
      51,     6,    61,     7,    -1,    60,    -1,    51,    62,    -1,
      61,    62,    -1,    62,    -1,    53,     5,    -1,    64,    65,
      -1,    -1,    66,    65,    -1,    -1,     3,    54,    -1,    67,
      68,    -1,    68,    -1,     8,     3,    63,    -1,     9,    10,
      -1,     9,    18,    -1,     9,    38,    -1,    11,     3,    63,
      -1,    12,    -1,    13,    -1,    14,     3,    63,    -1,    15,
      59,    -1,    16,     3,    -1,    17,     3,    -1,    19,     3,
      63,    -1,    20,    -1,    21,     3,    -1,    22,     3,    63,
      -1,    23,     5,     5,     5,     5,     5,     5,     5,     5,
       5,    -1,    24,     3,    63,    -1,    25,     3,    63,    -1,
      26,     3,    63,    -1,    27,     3,    -1,    27,     3,     3,
      -1,    28,     3,    -1,    29,     3,    -1,    30,    -1,    31,
       3,    -1,    32,     3,    63,    -1,    33,    -1,    34,     5,
       5,     5,     5,    -1,    35,     3,    63,    -1,    36,     5,
       5,     5,    -1,    37,     3,    63,    -1,    39,     3,    63,
      -1,    40,     3,     3,     3,    63,    -1,    41,    -1,    42,
      -1,    43,     5,     5,    -1,    44,    59,    -1,    45,     5,
       5,     5,    -1,    46,    -1,    47,    -1
};

/* YYRLINE[YYN] -- source line where rule number YYN was defined.  */
static const yytype_uint16 yyrline[] =
{
       0,   183,   183,   189,   197,   205,   213,   219,   226,   233,
     241,   247,   252,   258,   266,   273,   281,   287,   292,   298,
     306,   312,   325,   331,   336,   344,   349,   355,   364,   370,
     376,   382,   391,   397,   403,   412,   424,   430,   436,   445,
     451,   457,   466,   472,   481,   490,   499,   505,   511,   517,
     523,   529,   535,   544,   550,   556,   565,   571,   580,   589,
     598,   604,   610,   616,   628,   634,   640
};
#endif

#if YYDEBUG || YYERROR_VERBOSE || YYTOKEN_TABLE
/* YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
   First, the terminals, then, starting at YYNTOKENS, nonterminals.  */
static const char *const yytname[] =
{
  "$end", "error", "$undefined", "STRING", "ID", "NUM", "LBRACK",
  "RBRACK", "ACCELERATOR", "ACTIVETRANSFORM", "ALL", "AREALIGHTSOURCE",
  "ATTRIBUTEBEGIN", "ATTRIBUTEEND", "CAMERA", "CONCATTRANSFORM",
  "COORDINATESYSTEM", "COORDSYSTRANSFORM", "ENDTIME", "FILM", "IDENTITY",
  "INCLUDE", "LIGHTSOURCE", "LOOKAT", "MAKENAMEDMATERIAL",
  "MAKENAMEDMEDIUM", "MATERIAL", "MEDIUMINTERFACE", "NAMEDMATERIAL",
  "OBJECTBEGIN", "OBJECTEND", "OBJECTINSTANCE", "PIXELFILTER",
  "REVERSEORIENTATION", "ROTATE", "SAMPLER", "SCALE", "SHAPE", "STARTTIME",
  "INTEGRATOR", "TEXTURE", "TRANSFORMBEGIN", "TRANSFORMEND",
  "TRANSFORMTIMES", "TRANSFORM", "TRANSLATE", "WORLDBEGIN", "WORLDEND",
  "HIGH_PRECEDENCE", "$accept", "start", "array_init", "string_array_init",
  "num_array_init", "array", "string_array", "single_element_string_array",
  "string_list", "string_list_entry", "num_array",
  "single_element_num_array", "num_list", "num_list_entry", "paramlist",
  "paramlist_init", "paramlist_contents", "paramlist_entry",
  "pbrt_stmt_list", "pbrt_stmt", 0
};
#endif

# ifdef YYPRINT
/* YYTOKNUM[YYLEX-NUM] -- Internal token number corresponding to
   token YYLEX-NUM.  */
static const yytype_uint16 yytoknum[] =
{
       0,   256,   257,   258,   259,   260,   261,   262,   263,   264,
     265,   266,   267,   268,   269,   270,   271,   272,   273,   274,
     275,   276,   277,   278,   279,   280,   281,   282,   283,   284,
     285,   286,   287,   288,   289,   290,   291,   292,   293,   294,
     295,   296,   297,   298,   299,   300,   301,   302,   303
};
# endif

/* YYR1[YYN] -- Symbol number of symbol that rule YYN derives.  */
static const yytype_uint8 yyr1[] =
{
       0,    49,    50,    51,    52,    53,    54,    54,    55,    55,
      56,    57,    57,    58,    59,    59,    60,    61,    61,    62,
      63,    64,    65,    65,    66,    67,    67,    68,    68,    68,
      68,    68,    68,    68,    68,    68,    68,    68,    68,    68,
      68,    68,    68,    68,    68,    68,    68,    68,    68,    68,
      68,    68,    68,    68,    68,    68,    68,    68,    68,    68,
      68,    68,    68,    68,    68,    68,    68
};

/* YYR2[YYN] -- Number of symbols composing right hand side of rule YYN.  */
static const yytype_uint8 yyr2[] =
{
       0,     2,     1,     0,     0,     0,     1,     1,     4,     1,
       2,     2,     1,     2,     4,     1,     2,     2,     1,     2,
       2,     0,     2,     0,     2,     2,     1,     3,     2,     2,
       2,     3,     1,     1,     3,     2,     2,     2,     3,     1,
       2,     3,    10,     3,     3,     3,     2,     3,     2,     2,
       1,     2,     3,     1,     5,     3,     4,     3,     3,     5,
       1,     1,     3,     2,     4,     1,     1
};

/* YYDEFACT[STATE-NAME] -- Default rule to reduce with in state
   STATE-NUM when YYTABLE doesn't specify something else to do.  Zero
   means the default is an error.  */
static const yytype_uint8 yydefact[] =
{
       0,     0,     0,     0,    32,    33,     0,     3,     0,     0,
       0,    39,     0,     0,     0,     0,     0,     0,     0,     0,
       0,    50,     0,     0,    53,     0,     0,     0,     0,     0,
       0,    60,    61,     0,     3,     0,    65,    66,     0,     2,
      26,    21,    28,    29,    30,    21,    21,     5,    35,    15,
      36,    37,    21,    40,    21,     0,    21,    21,    21,    46,
      48,    49,    51,    21,     0,    21,     0,    21,    21,     0,
       0,    63,     0,     1,    25,    27,    23,    31,    34,     5,
       0,    16,    38,    41,     0,    43,    44,    45,    47,    52,
       0,    55,     0,    57,    58,     0,    62,     0,     3,    20,
      23,     5,    18,    19,     0,     0,    56,    21,    64,     4,
      24,     6,     9,     7,    22,    14,    17,     0,    54,    59,
       4,     0,    10,     0,     4,    12,    13,     0,     8,    11,
       0,     0,    42
};

/* YYDEFGOTO[NTERM-NUM].  */
static const yytype_int8 yydefgoto[] =
{
      -1,    38,    47,   121,    80,   110,   111,   112,   124,   122,
      48,    49,   101,    81,    75,    76,    99,   100,    39,    40
};

/* YYPACT[STATE-NUM] -- Index in YYTABLE of the portion describing
   STATE-NUM.  */
#define YYPACT_NINF -116
static const yytype_int8 yypact[] =
{
      57,     1,    -3,     3,  -116,  -116,     8,  -116,    15,    17,
      19,  -116,    23,    24,    25,    26,    28,    29,    30,    31,
      33,  -116,    34,    35,  -116,    36,    37,    38,    39,    42,
      43,  -116,  -116,    44,  -116,    45,  -116,  -116,    47,    57,
    -116,  -116,  -116,  -116,  -116,  -116,  -116,    22,  -116,  -116,
    -116,  -116,  -116,  -116,  -116,    46,  -116,  -116,  -116,    49,
    -116,  -116,  -116,  -116,    48,  -116,    50,  -116,  -116,    51,
      52,  -116,    53,  -116,  -116,  -116,    56,  -116,  -116,  -116,
      55,  -116,  -116,  -116,    62,  -116,  -116,  -116,  -116,  -116,
      70,  -116,    90,  -116,  -116,    58,  -116,   100,  -116,  -116,
      56,    32,  -116,  -116,   101,   102,  -116,  -116,  -116,    11,
    -116,  -116,  -116,  -116,  -116,  -116,  -116,   103,  -116,  -116,
     104,    59,  -116,   105,    41,  -116,  -116,   106,  -116,  -116,
     107,   108,  -116
};

/* YYPGOTO[NTERM-NUM].  */
static const yytype_int8 yypgoto[] =
{
    -116,  -116,   -42,  -116,  -116,  -116,  -116,  -116,  -116,  -115,
     -34,  -116,  -116,   -76,   -44,  -116,    14,  -116,  -116,    76
};

/* YYTABLE[YYPACT[STATE-NUM]].  What to do in state STATE-NUM.  If
   positive, shift that token.  If negative, reduce the rule which
   number is the opposite.  If zero, do what YYDEFACT says.
   If YYTABLE_NINF, syntax error.  */
#define YYTABLE_NINF -6
static const yytype_int16 yytable[] =
{
      71,    77,    78,   102,    41,   125,    45,    42,    82,   129,
      83,    46,    85,    86,    87,    43,    -5,   120,    50,    89,
      51,    91,    52,    93,    94,   116,    53,    54,    79,    56,
      55,    57,    58,    59,    60,    44,    61,    62,    63,   115,
      65,    64,    67,    66,   102,    68,    69,    73,   128,    70,
      72,    84,    88,    90,    95,    92,   109,    96,    97,    98,
     103,   107,   126,   119,   113,     1,     2,   104,     3,     4,
       5,     6,     7,     8,     9,   105,    10,    11,    12,    13,
      14,    15,    16,    17,    18,    19,    20,    21,    22,    23,
      24,    25,    26,    27,    28,   106,    29,    30,    31,    32,
      33,    34,    35,    36,    37,   108,   117,   118,   123,    -5,
     127,   130,   131,   132,   114,    74
};

static const yytype_uint8 yycheck[] =
{
      34,    45,    46,    79,     3,   120,     3,    10,    52,   124,
      54,     3,    56,    57,    58,    18,     5,     6,     3,    63,
       3,    65,     3,    67,    68,   101,     3,     3,     6,     3,
       5,     3,     3,     3,     3,    38,     3,     3,     3,     7,
       3,     5,     3,     5,   120,     3,     3,     0,     7,     5,
       5,     5,     3,     5,     3,     5,    98,     5,     5,     3,
       5,     3,     3,   107,    98,     8,     9,     5,    11,    12,
      13,    14,    15,    16,    17,     5,    19,    20,    21,    22,
      23,    24,    25,    26,    27,    28,    29,    30,    31,    32,
      33,    34,    35,    36,    37,     5,    39,    40,    41,    42,
      43,    44,    45,    46,    47,     5,     5,     5,     5,     5,
       5,     5,     5,     5,   100,    39
};

/* YYSTOS[STATE-NUM] -- The (internal number of the) accessing
   symbol of state STATE-NUM.  */
static const yytype_uint8 yystos[] =
{
       0,     8,     9,    11,    12,    13,    14,    15,    16,    17,
      19,    20,    21,    22,    23,    24,    25,    26,    27,    28,
      29,    30,    31,    32,    33,    34,    35,    36,    37,    39,
      40,    41,    42,    43,    44,    45,    46,    47,    50,    67,
      68,     3,    10,    18,    38,     3,     3,    51,    59,    60,
       3,     3,     3,     3,     3,     5,     3,     3,     3,     3,
       3,     3,     3,     3,     5,     3,     5,     3,     3,     3,
       5,    59,     5,     0,    68,    63,    64,    63,    63,     6,
      53,    62,    63,    63,     5,    63,    63,    63,     3,    63,
       5,    63,     5,    63,    63,     3,     5,     5,     3,    65,
      66,    61,    62,     5,     5,     5,     5,     3,     5,    51,
      54,    55,    56,    59,    65,     7,    62,     5,     5,    63,
       6,    52,    58,     5,    57,    58,     3,     5,     7,    58,
       5,     5,     5
};

#define yyerrok		(yyerrstatus = 0)
#define yyclearin	(yychar = YYEMPTY)
#define YYEMPTY		(-2)
#define YYEOF		0

#define YYACCEPT	goto yyacceptlab
#define YYABORT		goto yyabortlab
#define YYERROR		goto yyerrorlab


/* Like YYERROR except do call yyerror.  This remains here temporarily
   to ease the transition to the new meaning of YYERROR, for GCC.
   Once GCC version 2 has supplanted version 1, this can go.  */

#define YYFAIL		goto yyerrlab

#define YYRECOVERING()  (!!yyerrstatus)

#define YYBACKUP(Token, Value)					\
do								\
  if (yychar == YYEMPTY && yylen == 1)				\
    {								\
      yychar = (Token);						\
      yylval = (Value);						\
      yytoken = YYTRANSLATE (yychar);				\
      YYPOPSTACK (1);						\
      goto yybackup;						\
    }								\
  else								\
    {								\
      yyerror (YY_("syntax error: cannot back up")); \
      YYERROR;							\
    }								\
while (YYID (0))


#define YYTERROR	1
#define YYERRCODE	256


/* YYLLOC_DEFAULT -- Set CURRENT to span from RHS[1] to RHS[N].
   If N is 0, then set CURRENT to the empty location which ends
   the previous symbol: RHS[0] (always defined).  */

#define YYRHSLOC(Rhs, K) ((Rhs)[K])
#ifndef YYLLOC_DEFAULT
# define YYLLOC_DEFAULT(Current, Rhs, N)				\
    do									\
      if (YYID (N))                                                    \
	{								\
	  (Current).first_line   = YYRHSLOC (Rhs, 1).first_line;	\
	  (Current).first_column = YYRHSLOC (Rhs, 1).first_column;	\
	  (Current).last_line    = YYRHSLOC (Rhs, N).last_line;		\
	  (Current).last_column  = YYRHSLOC (Rhs, N).last_column;	\
	}								\
      else								\
	{								\
	  (Current).first_line   = (Current).last_line   =		\
	    YYRHSLOC (Rhs, 0).last_line;				\
	  (Current).first_column = (Current).last_column =		\
	    YYRHSLOC (Rhs, 0).last_column;				\
	}								\
    while (YYID (0))
#endif


/* YY_LOCATION_PRINT -- Print the location on the stream.
   This macro was not mandated originally: define only if we know
   we won't break user code: when these are the locations we know.  */

#ifndef YY_LOCATION_PRINT
# if defined YYLTYPE_IS_TRIVIAL && YYLTYPE_IS_TRIVIAL
#  define YY_LOCATION_PRINT(File, Loc)			\
     fprintf (File, "%d.%d-%d.%d",			\
	      (Loc).first_line, (Loc).first_column,	\
	      (Loc).last_line,  (Loc).last_column)
# else
#  define YY_LOCATION_PRINT(File, Loc) ((void) 0)
# endif
#endif


/* YYLEX -- calling `yylex' with the right arguments.  */

#ifdef YYLEX_PARAM
# define YYLEX yylex (YYLEX_PARAM)
#else
# define YYLEX yylex ()
#endif

/* Enable debugging if requested.  */
#if YYDEBUG

# ifndef YYFPRINTF
#  include <stdio.h> /* INFRINGES ON USER NAME SPACE */
#  define YYFPRINTF fprintf
# endif

# define YYDPRINTF(Args)			\
do {						\
  if (yydebug)					\
    YYFPRINTF Args;				\
} while (YYID (0))

# define YY_SYMBOL_PRINT(Title, Type, Value, Location)			  \
do {									  \
  if (yydebug)								  \
    {									  \
      YYFPRINTF (stderr, "%s ", Title);					  \
      yy_symbol_print (stderr,						  \
		  Type, Value); \
      YYFPRINTF (stderr, "\n");						  \
    }									  \
} while (YYID (0))


/*--------------------------------.
| Print this symbol on YYOUTPUT.  |
`--------------------------------*/

/*ARGSUSED*/
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_symbol_value_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep)
#else
static void
yy_symbol_value_print (yyoutput, yytype, yyvaluep)
    FILE *yyoutput;
    int yytype;
    YYSTYPE const * const yyvaluep;
#endif
{
  if (!yyvaluep)
    return;
# ifdef YYPRINT
  if (yytype < YYNTOKENS)
    YYPRINT (yyoutput, yytoknum[yytype], *yyvaluep);
# else
  YYUSE (yyoutput);
# endif
  switch (yytype)
    {
      default:
	break;
    }
}


/*--------------------------------.
| Print this symbol on YYOUTPUT.  |
`--------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_symbol_print (FILE *yyoutput, int yytype, YYSTYPE const * const yyvaluep)
#else
static void
yy_symbol_print (yyoutput, yytype, yyvaluep)
    FILE *yyoutput;
    int yytype;
    YYSTYPE const * const yyvaluep;
#endif
{
  if (yytype < YYNTOKENS)
    YYFPRINTF (yyoutput, "token %s (", yytname[yytype]);
  else
    YYFPRINTF (yyoutput, "nterm %s (", yytname[yytype]);

  yy_symbol_value_print (yyoutput, yytype, yyvaluep);
  YYFPRINTF (yyoutput, ")");
}

/*------------------------------------------------------------------.
| yy_stack_print -- Print the state stack from its BOTTOM up to its |
| TOP (included).                                                   |
`------------------------------------------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_stack_print (yytype_int16 *bottom, yytype_int16 *top)
#else
static void
yy_stack_print (bottom, top)
    yytype_int16 *bottom;
    yytype_int16 *top;
#endif
{
  YYFPRINTF (stderr, "Stack now");
  for (; bottom <= top; ++bottom)
    YYFPRINTF (stderr, " %d", *bottom);
  YYFPRINTF (stderr, "\n");
}

# define YY_STACK_PRINT(Bottom, Top)				\
do {								\
  if (yydebug)							\
    yy_stack_print ((Bottom), (Top));				\
} while (YYID (0))


/*------------------------------------------------.
| Report that the YYRULE is going to be reduced.  |
`------------------------------------------------*/

#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yy_reduce_print (YYSTYPE *yyvsp, int yyrule)
#else
static void
yy_reduce_print (yyvsp, yyrule)
    YYSTYPE *yyvsp;
    int yyrule;
#endif
{
  int yynrhs = yyr2[yyrule];
  int yyi;
  unsigned long int yylno = yyrline[yyrule];
  YYFPRINTF (stderr, "Reducing stack by rule %d (line %lu):\n",
	     yyrule - 1, yylno);
  /* The symbols being reduced.  */
  for (yyi = 0; yyi < yynrhs; yyi++)
    {
      fprintf (stderr, "   $%d = ", yyi + 1);
      yy_symbol_print (stderr, yyrhs[yyprhs[yyrule] + yyi],
		       &(yyvsp[(yyi + 1) - (yynrhs)])
		       		       );
      fprintf (stderr, "\n");
    }
}

# define YY_REDUCE_PRINT(Rule)		\
do {					\
  if (yydebug)				\
    yy_reduce_print (yyvsp, Rule); \
} while (YYID (0))

/* Nonzero means print parse trace.  It is left uninitialized so that
   multiple parsers can coexist.  */
int yydebug;
#else /* !YYDEBUG */
# define YYDPRINTF(Args)
# define YY_SYMBOL_PRINT(Title, Type, Value, Location)
# define YY_STACK_PRINT(Bottom, Top)
# define YY_REDUCE_PRINT(Rule)
#endif /* !YYDEBUG */


/* YYINITDEPTH -- initial size of the parser's stacks.  */
#ifndef	YYINITDEPTH
# define YYINITDEPTH 200
#endif

/* YYMAXDEPTH -- maximum size the stacks can grow to (effective only
   if the built-in stack extension method is used).

   Do not make this value too large; the results are undefined if
   YYSTACK_ALLOC_MAXIMUM < YYSTACK_BYTES (YYMAXDEPTH)
   evaluated with infinite-precision integer arithmetic.  */

#ifndef YYMAXDEPTH
# define YYMAXDEPTH 10000
#endif



#if YYERROR_VERBOSE

# ifndef yystrlen
#  if defined __GLIBC__ && defined _STRING_H
#   define yystrlen strlen
#  else
/* Return the length of YYSTR.  */
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static YYSIZE_T
yystrlen (const char *yystr)
#else
static YYSIZE_T
yystrlen (yystr)
    const char *yystr;
#endif
{
  YYSIZE_T yylen;
  for (yylen = 0; yystr[yylen]; yylen++)
    continue;
  return yylen;
}
#  endif
# endif

# ifndef yystpcpy
#  if defined __GLIBC__ && defined _STRING_H && defined _GNU_SOURCE
#   define yystpcpy stpcpy
#  else
/* Copy YYSRC to YYDEST, returning the address of the terminating '\0' in
   YYDEST.  */
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static char *
yystpcpy (char *yydest, const char *yysrc)
#else
static char *
yystpcpy (yydest, yysrc)
    char *yydest;
    const char *yysrc;
#endif
{
  char *yyd = yydest;
  const char *yys = yysrc;

  while ((*yyd++ = *yys++) != '\0')
    continue;

  return yyd - 1;
}
#  endif
# endif

# ifndef yytnamerr
/* Copy to YYRES the contents of YYSTR after stripping away unnecessary
   quotes and backslashes, so that it's suitable for yyerror.  The
   heuristic is that double-quoting is unnecessary unless the string
   contains an apostrophe, a comma, or backslash (other than
   backslash-backslash).  YYSTR is taken from yytname.  If YYRES is
   null, do not copy; instead, return the length of what the result
   would have been.  */
static YYSIZE_T
yytnamerr (char *yyres, const char *yystr)
{
  if (*yystr == '"')
    {
      YYSIZE_T yyn = 0;
      char const *yyp = yystr;

      for (;;)
	switch (*++yyp)
	  {
	  case '\'':
	  case ',':
	    goto do_not_strip_quotes;

	  case '\\':
	    if (*++yyp != '\\')
	      goto do_not_strip_quotes;
	    /* Fall through.  */
	  default:
	    if (yyres)
	      yyres[yyn] = *yyp;
	    yyn++;
	    break;

	  case '"':
	    if (yyres)
	      yyres[yyn] = '\0';
	    return yyn;
	  }
    do_not_strip_quotes: ;
    }

  if (! yyres)
    return yystrlen (yystr);

  return yystpcpy (yyres, yystr) - yyres;
}
# endif

/* Copy into YYRESULT an error message about the unexpected token
   YYCHAR while in state YYSTATE.  Return the number of bytes copied,
   including the terminating null byte.  If YYRESULT is null, do not
   copy anything; just return the number of bytes that would be
   copied.  As a special case, return 0 if an ordinary "syntax error"
   message will do.  Return YYSIZE_MAXIMUM if overflow occurs during
   size calculation.  */
static YYSIZE_T
yysyntax_error (char *yyresult, int yystate, int yychar)
{
  int yyn = yypact[yystate];

  if (! (YYPACT_NINF < yyn && yyn <= YYLAST))
    return 0;
  else
    {
      int yytype = YYTRANSLATE (yychar);
      YYSIZE_T yysize0 = yytnamerr (0, yytname[yytype]);
      YYSIZE_T yysize = yysize0;
      YYSIZE_T yysize1;
      int yysize_overflow = 0;
      enum { YYERROR_VERBOSE_ARGS_MAXIMUM = 5 };
      char const *yyarg[YYERROR_VERBOSE_ARGS_MAXIMUM];
      int yyx;

# if 0
      /* This is so xgettext sees the translatable formats that are
	 constructed on the fly.  */
      YY_("syntax error, unexpected %s");
      YY_("syntax error, unexpected %s, expecting %s");
      YY_("syntax error, unexpected %s, expecting %s or %s");
      YY_("syntax error, unexpected %s, expecting %s or %s or %s");
      YY_("syntax error, unexpected %s, expecting %s or %s or %s or %s");
# endif
      char *yyfmt;
      char const *yyf;
      static char const yyunexpected[] = "syntax error, unexpected %s";
      static char const yyexpecting[] = ", expecting %s";
      static char const yyor[] = " or %s";
      char yyformat[sizeof yyunexpected
		    + sizeof yyexpecting - 1
		    + ((YYERROR_VERBOSE_ARGS_MAXIMUM - 2)
		       * (sizeof yyor - 1))];
      char const *yyprefix = yyexpecting;

      /* Start YYX at -YYN if negative to avoid negative indexes in
	 YYCHECK.  */
      int yyxbegin = yyn < 0 ? -yyn : 0;

      /* Stay within bounds of both yycheck and yytname.  */
      int yychecklim = YYLAST - yyn + 1;
      int yyxend = yychecklim < YYNTOKENS ? yychecklim : YYNTOKENS;
      int yycount = 1;

      yyarg[0] = yytname[yytype];
      yyfmt = yystpcpy (yyformat, yyunexpected);

      for (yyx = yyxbegin; yyx < yyxend; ++yyx)
	if (yycheck[yyx + yyn] == yyx && yyx != YYTERROR)
	  {
	    if (yycount == YYERROR_VERBOSE_ARGS_MAXIMUM)
	      {
		yycount = 1;
		yysize = yysize0;
		yyformat[sizeof yyunexpected - 1] = '\0';
		break;
	      }
	    yyarg[yycount++] = yytname[yyx];
	    yysize1 = yysize + yytnamerr (0, yytname[yyx]);
	    yysize_overflow |= (yysize1 < yysize);
	    yysize = yysize1;
	    yyfmt = yystpcpy (yyfmt, yyprefix);
	    yyprefix = yyor;
	  }

      yyf = YY_(yyformat);
      yysize1 = yysize + yystrlen (yyf);
      yysize_overflow |= (yysize1 < yysize);
      yysize = yysize1;

      if (yysize_overflow)
	return YYSIZE_MAXIMUM;

      if (yyresult)
	{
	  /* Avoid sprintf, as that infringes on the user's name space.
	     Don't have undefined behavior even if the translation
	     produced a string with the wrong number of "%s"s.  */
	  char *yyp = yyresult;
	  int yyi = 0;
	  while ((*yyp = *yyf) != '\0')
	    {
	      if (*yyp == '%' && yyf[1] == 's' && yyi < yycount)
		{
		  yyp += yytnamerr (yyp, yyarg[yyi++]);
		  yyf += 2;
		}
	      else
		{
		  yyp++;
		  yyf++;
		}
	    }
	}
      return yysize;
    }
}
#endif /* YYERROR_VERBOSE */


/*-----------------------------------------------.
| Release the memory associated to this symbol.  |
`-----------------------------------------------*/

/*ARGSUSED*/
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
static void
yydestruct (const char *yymsg, int yytype, YYSTYPE *yyvaluep)
#else
static void
yydestruct (yymsg, yytype, yyvaluep)
    const char *yymsg;
    int yytype;
    YYSTYPE *yyvaluep;
#endif
{
  YYUSE (yyvaluep);

  if (!yymsg)
    yymsg = "Deleting";
  YY_SYMBOL_PRINT (yymsg, yytype, yyvaluep, yylocationp);

  switch (yytype)
    {

      default:
	break;
    }
}


/* Prevent warnings from -Wmissing-prototypes.  */

#ifdef YYPARSE_PARAM
#if defined __STDC__ || defined __cplusplus
int yyparse (void *YYPARSE_PARAM);
#else
int yyparse ();
#endif
#else /* ! YYPARSE_PARAM */
#if defined __STDC__ || defined __cplusplus
int yyparse (void);
#else
int yyparse ();
#endif
#endif /* ! YYPARSE_PARAM */



/* The look-ahead symbol.  */
int yychar;

/* The semantic value of the look-ahead symbol.  */
YYSTYPE yylval;

/* Number of syntax errors so far.  */
int yynerrs;



/*----------.
| yyparse.  |
`----------*/

#ifdef YYPARSE_PARAM
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
int
yyparse (void *YYPARSE_PARAM)
#else
int
yyparse (YYPARSE_PARAM)
    void *YYPARSE_PARAM;
#endif
#else /* ! YYPARSE_PARAM */
#if (defined __STDC__ || defined __C99__FUNC__ \
     || defined __cplusplus || defined _MSC_VER)
int
yyparse (void)
#else
int
yyparse ()

#endif
#endif
{
  
  int yystate;
  int yyn;
  int yyresult;
  /* Number of tokens to shift before error messages enabled.  */
  int yyerrstatus;
  /* Look-ahead token as an internal (translated) token number.  */
  int yytoken = 0;
#if YYERROR_VERBOSE
  /* Buffer for error messages, and its allocated size.  */
  char yymsgbuf[128];
  char *yymsg = yymsgbuf;
  YYSIZE_T yymsg_alloc = sizeof yymsgbuf;
#endif

  /* Three stacks and their tools:
     `yyss': related to states,
     `yyvs': related to semantic values,
     `yyls': related to locations.

     Refer to the stacks thru separate pointers, to allow yyoverflow
     to reallocate them elsewhere.  */

  /* The state stack.  */
  yytype_int16 yyssa[YYINITDEPTH];
  yytype_int16 *yyss = yyssa;
  yytype_int16 *yyssp;

  /* The semantic value stack.  */
  YYSTYPE yyvsa[YYINITDEPTH];
  YYSTYPE *yyvs = yyvsa;
  YYSTYPE *yyvsp;



#define YYPOPSTACK(N)   (yyvsp -= (N), yyssp -= (N))

  YYSIZE_T yystacksize = YYINITDEPTH;

  /* The variables used to return semantic value and location from the
     action routines.  */
  YYSTYPE yyval;


  /* The number of symbols on the RHS of the reduced rule.
     Keep to zero when no symbol should be popped.  */
  int yylen = 0;

  YYDPRINTF ((stderr, "Starting parse\n"));

  yystate = 0;
  yyerrstatus = 0;
  yynerrs = 0;
  yychar = YYEMPTY;		/* Cause a token to be read.  */

  /* Initialize stack pointers.
     Waste one element of value and location stack
     so that they stay on the same level as the state stack.
     The wasted elements are never initialized.  */

  yyssp = yyss;
  yyvsp = yyvs;

  goto yysetstate;

/*------------------------------------------------------------.
| yynewstate -- Push a new state, which is found in yystate.  |
`------------------------------------------------------------*/
 yynewstate:
  /* In all cases, when you get here, the value and location stacks
     have just been pushed.  So pushing a state here evens the stacks.  */
  yyssp++;

 yysetstate:
  *yyssp = yystate;

  if (yyss + yystacksize - 1 <= yyssp)
    {
      /* Get the current used size of the three stacks, in elements.  */
      YYSIZE_T yysize = yyssp - yyss + 1;

#ifdef yyoverflow
      {
	/* Give user a chance to reallocate the stack.  Use copies of
	   these so that the &'s don't force the real ones into
	   memory.  */
	YYSTYPE *yyvs1 = yyvs;
	yytype_int16 *yyss1 = yyss;


	/* Each stack pointer address is followed by the size of the
	   data in use in that stack, in bytes.  This used to be a
	   conditional around just the two extra args, but that might
	   be undefined if yyoverflow is a macro.  */
	yyoverflow (YY_("memory exhausted"),
		    &yyss1, yysize * sizeof (*yyssp),
		    &yyvs1, yysize * sizeof (*yyvsp),

		    &yystacksize);

	yyss = yyss1;
	yyvs = yyvs1;
      }
#else /* no yyoverflow */
# ifndef YYSTACK_RELOCATE
      goto yyexhaustedlab;
# else
      /* Extend the stack our own way.  */
      if (YYMAXDEPTH <= yystacksize)
	goto yyexhaustedlab;
      yystacksize *= 2;
      if (YYMAXDEPTH < yystacksize)
	yystacksize = YYMAXDEPTH;

      {
	yytype_int16 *yyss1 = yyss;
	union yyalloc *yyptr =
	  (union yyalloc *) YYSTACK_ALLOC (YYSTACK_BYTES (yystacksize));
	if (! yyptr)
	  goto yyexhaustedlab;
	YYSTACK_RELOCATE (yyss);
	YYSTACK_RELOCATE (yyvs);

#  undef YYSTACK_RELOCATE
	if (yyss1 != yyssa)
	  YYSTACK_FREE (yyss1);
      }
# endif
#endif /* no yyoverflow */

      yyssp = yyss + yysize - 1;
      yyvsp = yyvs + yysize - 1;


      YYDPRINTF ((stderr, "Stack size increased to %lu\n",
		  (unsigned long int) yystacksize));

      if (yyss + yystacksize - 1 <= yyssp)
	YYABORT;
    }

  YYDPRINTF ((stderr, "Entering state %d\n", yystate));

  goto yybackup;

/*-----------.
| yybackup.  |
`-----------*/
yybackup:

  /* Do appropriate processing given the current state.  Read a
     look-ahead token if we need one and don't already have one.  */

  /* First try to decide what to do without reference to look-ahead token.  */
  yyn = yypact[yystate];
  if (yyn == YYPACT_NINF)
    goto yydefault;

  /* Not known => get a look-ahead token if don't already have one.  */

  /* YYCHAR is either YYEMPTY or YYEOF or a valid look-ahead symbol.  */
  if (yychar == YYEMPTY)
    {
      YYDPRINTF ((stderr, "Reading a token: "));
      yychar = YYLEX;
    }

  if (yychar <= YYEOF)
    {
      yychar = yytoken = YYEOF;
      YYDPRINTF ((stderr, "Now at end of input.\n"));
    }
  else
    {
      yytoken = YYTRANSLATE (yychar);
      YY_SYMBOL_PRINT ("Next token is", yytoken, &yylval, &yylloc);
    }

  /* If the proper action on seeing token YYTOKEN is to reduce or to
     detect an error, take that action.  */
  yyn += yytoken;
  if (yyn < 0 || YYLAST < yyn || yycheck[yyn] != yytoken)
    goto yydefault;
  yyn = yytable[yyn];
  if (yyn <= 0)
    {
      if (yyn == 0 || yyn == YYTABLE_NINF)
	goto yyerrlab;
      yyn = -yyn;
      goto yyreduce;
    }

  if (yyn == YYFINAL)
    YYACCEPT;

  /* Count tokens shifted since error; after three, turn off error
     status.  */
  if (yyerrstatus)
    yyerrstatus--;

  /* Shift the look-ahead token.  */
  YY_SYMBOL_PRINT ("Shifting", yytoken, &yylval, &yylloc);

  /* Discard the shifted token unless it is eof.  */
  if (yychar != YYEOF)
    yychar = YYEMPTY;

  yystate = yyn;
  *++yyvsp = yylval;

  goto yynewstate;


/*-----------------------------------------------------------.
| yydefault -- do the default action for the current state.  |
`-----------------------------------------------------------*/
yydefault:
  yyn = yydefact[yystate];
  if (yyn == 0)
    goto yyerrlab;
  goto yyreduce;


/*-----------------------------.
| yyreduce -- Do a reduction.  |
`-----------------------------*/
yyreduce:
  /* yyn is the number of a rule to reduce with.  */
  yylen = yyr2[yyn];

  /* If YYLEN is nonzero, implement the default value of the action:
     `$$ = $1'.

     Otherwise, the following line sets YYVAL to garbage.
     This behavior is undocumented and Bison
     users should not rely upon it.  Assigning to YYVAL
     unconditionally makes the parser a bit smaller, and it avoids a
     GCC warning that YYVAL may be used uninitialized.  */
  yyval = yyvsp[1-yylen];


  YY_REDUCE_PRINT (yyn);
  switch (yyn)
    {
        case 2:
#line 184 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
;}
    break;

  case 3:
#line 190 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    if (cur_array) LOG(FATAL) << "Unexpected error parsing array";
    cur_array = new ParamArray;
;}
    break;

  case 4:
#line 198 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    cur_array->element_size = sizeof(const char *);
    cur_array->isString = true;
;}
    break;

  case 5:
#line 206 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    cur_array->element_size = sizeof(double);
    cur_array->isString = false;
;}
    break;

  case 6:
#line 214 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    (yyval.ribarray) = (yyvsp[(1) - (1)].ribarray);
;}
    break;

  case 7:
#line 220 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    (yyval.ribarray) = (yyvsp[(1) - (1)].ribarray);
;}
    break;

  case 8:
#line 227 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    (yyval.ribarray) = cur_array;
    cur_array = nullptr;
;}
    break;

  case 9:
#line 234 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    (yyval.ribarray) = cur_array;
    cur_array = nullptr;
;}
    break;

  case 10:
#line 242 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
;}
    break;

  case 11:
#line 248 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
;}
    break;

  case 12:
#line 253 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
;}
    break;

  case 13:
#line 259 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    char *to_add = strdup((yyvsp[(2) - (2)].string));
    AddArrayElement(&to_add);
;}
    break;

  case 14:
#line 267 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    (yyval.ribarray) = cur_array;
    cur_array = nullptr;
;}
    break;

  case 15:
#line 274 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    (yyval.ribarray) = cur_array;
    cur_array = nullptr;
;}
    break;

  case 16:
#line 282 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
;}
    break;

  case 17:
#line 288 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
;}
    break;

  case 18:
#line 293 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
;}
    break;

  case 19:
#line 299 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    double to_add = (yyvsp[(2) - (2)].num);
    AddArrayElement(&to_add);
;}
    break;

  case 20:
#line 307 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
;}
    break;

  case 21:
#line 313 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    for (size_t i = 0; i < cur_paramlist.size(); ++i) {
        if (cur_paramlist[i].isString) {
            for (size_t j = 0; j < cur_paramlist[i].size; ++j)
                free(((char **)cur_paramlist[i].arg)[j]);
        }
    }
    cur_paramlist.erase(cur_paramlist.begin(), cur_paramlist.end());
;}
    break;

  case 22:
#line 326 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
;}
    break;

  case 23:
#line 331 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
;}
    break;

  case 24:
#line 337 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    cur_paramlist.push_back(ParamListItem((yyvsp[(1) - (2)].string), (yyvsp[(2) - (2)].ribarray)));
    ArrayFree((yyvsp[(2) - (2)].ribarray));
;}
    break;

  case 25:
#line 345 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
;}
    break;

  case 26:
#line 350 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
;}
    break;

  case 27:
#line 356 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    ParamSet params;
    InitParamSet(params, SpectrumType::Reflectance);
    pbrtAccelerator((yyvsp[(2) - (3)].string), params);
    FreeArgs();
;}
    break;

  case 28:
#line 365 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtActiveTransformAll();
;}
    break;

  case 29:
#line 371 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtActiveTransformEndTime();
;}
    break;

  case 30:
#line 377 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtActiveTransformStartTime();
;}
    break;

  case 31:
#line 383 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    ParamSet params;
    InitParamSet(params, SpectrumType::Illuminant);
    pbrtAreaLightSource((yyvsp[(2) - (3)].string), params);
    FreeArgs();
;}
    break;

  case 32:
#line 392 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtAttributeBegin();
;}
    break;

  case 33:
#line 398 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtAttributeEnd();
;}
    break;

  case 34:
#line 404 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    ParamSet params;
    InitParamSet(params, SpectrumType::Reflectance);
    pbrtCamera((yyvsp[(2) - (3)].string), params);
    FreeArgs();
;}
    break;

  case 35:
#line 413 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    if (VerifyArrayLength((yyvsp[(2) - (2)].ribarray), 16, "ConcatTransform")) {
        Float m[16];
        double *dm = (double *)(yyvsp[(2) - (2)].ribarray)->array;
        std::copy(dm, dm + 16, m);
        pbrtConcatTransform(m);
    }
    ArrayFree((yyvsp[(2) - (2)].ribarray));
;}
    break;

  case 36:
#line 425 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtCoordinateSystem((yyvsp[(2) - (2)].string));
;}
    break;

  case 37:
#line 431 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtCoordSysTransform((yyvsp[(2) - (2)].string));
;}
    break;

  case 38:
#line 437 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    ParamSet params;
    InitParamSet(params, SpectrumType::Reflectance);
    pbrtFilm((yyvsp[(2) - (3)].string), params);
    FreeArgs();
;}
    break;

  case 39:
#line 446 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtIdentity();
;}
    break;

  case 40:
#line 452 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
  include_push((yyvsp[(2) - (2)].string));
;}
    break;

  case 41:
#line 458 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    ParamSet params;
    InitParamSet(params, SpectrumType::Illuminant);
    pbrtLightSource((yyvsp[(2) - (3)].string), params);
    FreeArgs();
;}
    break;

  case 42:
#line 467 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtLookAt((yyvsp[(2) - (10)].num), (yyvsp[(3) - (10)].num), (yyvsp[(4) - (10)].num), (yyvsp[(5) - (10)].num), (yyvsp[(6) - (10)].num), (yyvsp[(7) - (10)].num), (yyvsp[(8) - (10)].num), (yyvsp[(9) - (10)].num), (yyvsp[(10) - (10)].num));
;}
    break;

  case 43:
#line 473 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    ParamSet params;
    InitParamSet(params, SpectrumType::Reflectance);
    pbrtMakeNamedMaterial((yyvsp[(2) - (3)].string), params);
    FreeArgs();
;}
    break;

  case 44:
#line 482 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    ParamSet params;
    InitParamSet(params, SpectrumType::Reflectance);
    pbrtMakeNamedMedium((yyvsp[(2) - (3)].string), params);
    FreeArgs();
;}
    break;

  case 45:
#line 491 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    ParamSet params;
    InitParamSet(params, SpectrumType::Reflectance);
    pbrtMaterial((yyvsp[(2) - (3)].string), params);
    FreeArgs();
;}
    break;

  case 46:
#line 500 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtMediumInterface((yyvsp[(2) - (2)].string), (yyvsp[(2) - (2)].string));
;}
    break;

  case 47:
#line 506 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtMediumInterface((yyvsp[(2) - (3)].string), (yyvsp[(3) - (3)].string));
;}
    break;

  case 48:
#line 512 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtNamedMaterial((yyvsp[(2) - (2)].string));
;}
    break;

  case 49:
#line 518 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtObjectBegin((yyvsp[(2) - (2)].string));
;}
    break;

  case 50:
#line 524 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtObjectEnd();
;}
    break;

  case 51:
#line 530 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtObjectInstance((yyvsp[(2) - (2)].string));
;}
    break;

  case 52:
#line 536 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    ParamSet params;
    InitParamSet(params, SpectrumType::Reflectance);
    pbrtPixelFilter((yyvsp[(2) - (3)].string), params);
    FreeArgs();
;}
    break;

  case 53:
#line 545 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtReverseOrientation();
;}
    break;

  case 54:
#line 551 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtRotate((yyvsp[(2) - (5)].num), (yyvsp[(3) - (5)].num), (yyvsp[(4) - (5)].num), (yyvsp[(5) - (5)].num));
;}
    break;

  case 55:
#line 557 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    ParamSet params;
    InitParamSet(params, SpectrumType::Reflectance);
    pbrtSampler((yyvsp[(2) - (3)].string), params);
    FreeArgs();
;}
    break;

  case 56:
#line 566 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtScale((yyvsp[(2) - (4)].num), (yyvsp[(3) - (4)].num), (yyvsp[(4) - (4)].num));
;}
    break;

  case 57:
#line 572 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    ParamSet params;
    InitParamSet(params, SpectrumType::Reflectance);
    pbrtShape((yyvsp[(2) - (3)].string), params);
    FreeArgs();
;}
    break;

  case 58:
#line 581 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    ParamSet params;
    InitParamSet(params, SpectrumType::Reflectance);
    pbrtIntegrator((yyvsp[(2) - (3)].string), params);
    FreeArgs();
;}
    break;

  case 59:
#line 590 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    ParamSet params;
    InitParamSet(params, SpectrumType::Reflectance);
    pbrtTexture((yyvsp[(2) - (5)].string), (yyvsp[(3) - (5)].string), (yyvsp[(4) - (5)].string), params);
    FreeArgs();
;}
    break;

  case 60:
#line 599 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtTransformBegin();
;}
    break;

  case 61:
#line 605 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtTransformEnd();
;}
    break;

  case 62:
#line 611 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtTransformTimes((yyvsp[(2) - (3)].num), (yyvsp[(3) - (3)].num));
;}
    break;

  case 63:
#line 617 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    if (VerifyArrayLength((yyvsp[(2) - (2)].ribarray), 16, "Transform")) {
        Float m[16];
        double *dm = (double *)(yyvsp[(2) - (2)].ribarray)->array;
        std::copy(dm, dm + 16, m);
        pbrtTransform(m);
    }
    ArrayFree((yyvsp[(2) - (2)].ribarray));
;}
    break;

  case 64:
#line 629 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtTranslate((yyvsp[(2) - (4)].num), (yyvsp[(3) - (4)].num), (yyvsp[(4) - (4)].num));
;}
    break;

  case 65:
#line 635 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtWorldBegin();
;}
    break;

  case 66:
#line 641 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"
    {
    pbrtWorldEnd();
;}
    break;


/* Line 1267 of yacc.c.  */
#line 2156 "/Users/mmp/build/pbrt-v3/pbrtparse.cpp"
      default: break;
    }
  YY_SYMBOL_PRINT ("-> $$ =", yyr1[yyn], &yyval, &yyloc);

  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);

  *++yyvsp = yyval;


  /* Now `shift' the result of the reduction.  Determine what state
     that goes to, based on the state we popped back to and the rule
     number reduced by.  */

  yyn = yyr1[yyn];

  yystate = yypgoto[yyn - YYNTOKENS] + *yyssp;
  if (0 <= yystate && yystate <= YYLAST && yycheck[yystate] == *yyssp)
    yystate = yytable[yystate];
  else
    yystate = yydefgoto[yyn - YYNTOKENS];

  goto yynewstate;


/*------------------------------------.
| yyerrlab -- here on detecting error |
`------------------------------------*/
yyerrlab:
  /* If not already recovering from an error, report this error.  */
  if (!yyerrstatus)
    {
      ++yynerrs;
#if ! YYERROR_VERBOSE
      yyerror (YY_("syntax error"));
#else
      {
	YYSIZE_T yysize = yysyntax_error (0, yystate, yychar);
	if (yymsg_alloc < yysize && yymsg_alloc < YYSTACK_ALLOC_MAXIMUM)
	  {
	    YYSIZE_T yyalloc = 2 * yysize;
	    if (! (yysize <= yyalloc && yyalloc <= YYSTACK_ALLOC_MAXIMUM))
	      yyalloc = YYSTACK_ALLOC_MAXIMUM;
	    if (yymsg != yymsgbuf)
	      YYSTACK_FREE (yymsg);
	    yymsg = (char *) YYSTACK_ALLOC (yyalloc);
	    if (yymsg)
	      yymsg_alloc = yyalloc;
	    else
	      {
		yymsg = yymsgbuf;
		yymsg_alloc = sizeof yymsgbuf;
	      }
	  }

	if (0 < yysize && yysize <= yymsg_alloc)
	  {
	    (void) yysyntax_error (yymsg, yystate, yychar);
	    yyerror (yymsg);
	  }
	else
	  {
	    yyerror (YY_("syntax error"));
	    if (yysize != 0)
	      goto yyexhaustedlab;
	  }
      }
#endif
    }



  if (yyerrstatus == 3)
    {
      /* If just tried and failed to reuse look-ahead token after an
	 error, discard it.  */

      if (yychar <= YYEOF)
	{
	  /* Return failure if at end of input.  */
	  if (yychar == YYEOF)
	    YYABORT;
	}
      else
	{
	  yydestruct ("Error: discarding",
		      yytoken, &yylval);
	  yychar = YYEMPTY;
	}
    }

  /* Else will try to reuse look-ahead token after shifting the error
     token.  */
  goto yyerrlab1;


/*---------------------------------------------------.
| yyerrorlab -- error raised explicitly by YYERROR.  |
`---------------------------------------------------*/
yyerrorlab:

  /* Pacify compilers like GCC when the user code never invokes
     YYERROR and the label yyerrorlab therefore never appears in user
     code.  */
  if (/*CONSTCOND*/ 0)
     goto yyerrorlab;

  /* Do not reclaim the symbols of the rule which action triggered
     this YYERROR.  */
  YYPOPSTACK (yylen);
  yylen = 0;
  YY_STACK_PRINT (yyss, yyssp);
  yystate = *yyssp;
  goto yyerrlab1;


/*-------------------------------------------------------------.
| yyerrlab1 -- common code for both syntax error and YYERROR.  |
`-------------------------------------------------------------*/
yyerrlab1:
  yyerrstatus = 3;	/* Each real token shifted decrements this.  */

  for (;;)
    {
      yyn = yypact[yystate];
      if (yyn != YYPACT_NINF)
	{
	  yyn += YYTERROR;
	  if (0 <= yyn && yyn <= YYLAST && yycheck[yyn] == YYTERROR)
	    {
	      yyn = yytable[yyn];
	      if (0 < yyn)
		break;
	    }
	}

      /* Pop the current state because it cannot handle the error token.  */
      if (yyssp == yyss)
	YYABORT;


      yydestruct ("Error: popping",
		  yystos[yystate], yyvsp);
      YYPOPSTACK (1);
      yystate = *yyssp;
      YY_STACK_PRINT (yyss, yyssp);
    }

  if (yyn == YYFINAL)
    YYACCEPT;

  *++yyvsp = yylval;


  /* Shift the error token.  */
  YY_SYMBOL_PRINT ("Shifting", yystos[yyn], yyvsp, yylsp);

  yystate = yyn;
  goto yynewstate;


/*-------------------------------------.
| yyacceptlab -- YYACCEPT comes here.  |
`-------------------------------------*/
yyacceptlab:
  yyresult = 0;
  goto yyreturn;

/*-----------------------------------.
| yyabortlab -- YYABORT comes here.  |
`-----------------------------------*/
yyabortlab:
  yyresult = 1;
  goto yyreturn;

#ifndef yyoverflow
/*-------------------------------------------------.
| yyexhaustedlab -- memory exhaustion comes here.  |
`-------------------------------------------------*/
yyexhaustedlab:
  yyerror (YY_("memory exhausted"));
  yyresult = 2;
  /* Fall through.  */
#endif

yyreturn:
  if (yychar != YYEOF && yychar != YYEMPTY)
     yydestruct ("Cleanup: discarding lookahead",
		 yytoken, &yylval);
  /* Do not reclaim the symbols of the rule which action triggered
     this YYABORT or YYACCEPT.  */
  YYPOPSTACK (yylen);
  YY_STACK_PRINT (yyss, yyssp);
  while (yyssp != yyss)
    {
      yydestruct ("Cleanup: popping",
		  yystos[*yyssp], yyvsp);
      YYPOPSTACK (1);
    }
#ifndef yyoverflow
  if (yyss != yyssa)
    YYSTACK_FREE (yyss);
#endif
#if YYERROR_VERBOSE
  if (yymsg != yymsgbuf)
    YYSTACK_FREE (yymsg);
#endif
  /* Make sure YYID is used.  */
  return YYID (yyresult);
}


#line 646 "/Users/mmp/pbrt-v3/src/core/pbrtparse.yy"

static const char *paramTypeToName(int type) {
    switch (type) {
    case PARAM_TYPE_INT: return "int";
    case PARAM_TYPE_BOOL: return "bool";
    case PARAM_TYPE_FLOAT: return "float";
    case PARAM_TYPE_POINT2: return "point2";
    case PARAM_TYPE_VECTOR2: return "vector2";
    case PARAM_TYPE_POINT3: return "point3";
    case PARAM_TYPE_VECTOR3: return "vector3";
    case PARAM_TYPE_NORMAL: return "normal";
    case PARAM_TYPE_RGB: return "rgb/color";
    case PARAM_TYPE_XYZ: return "xyz";
    case PARAM_TYPE_BLACKBODY: return "blackbody";
    case PARAM_TYPE_SPECTRUM: return "spectrum";
    case PARAM_TYPE_STRING: return "string";
    case PARAM_TYPE_TEXTURE: return "texture";
    default: LOG(FATAL) << "Error in paramTypeToName"; return nullptr;
    }
}


static void InitParamSet(ParamSet &ps, SpectrumType type) {
    ps.Clear();
    for (size_t i = 0; i < cur_paramlist.size(); ++i) {
        int type;
        std::string name;
        if (lookupType(cur_paramlist[i].name, &type, name)) {
            if (type == PARAM_TYPE_TEXTURE || type == PARAM_TYPE_STRING ||
                type == PARAM_TYPE_BOOL) {
                if (!cur_paramlist[i].isString) {
                    Error("Expected string parameter value for parameter "
                          "\"%s\" with type \"%s\". Ignoring.",
                          name.c_str(), paramTypeToName(type));
                    continue;
                }
            }
            else if (type != PARAM_TYPE_SPECTRUM) { /* spectrum can be either... */
                if (cur_paramlist[i].isString) {
                    Error("Expected numeric parameter value for parameter "
                          "\"%s\" with type \"%s\".  Ignoring.",
                          name.c_str(), paramTypeToName(type));
                    continue;
                }
            }
            void *data = cur_paramlist[i].arg;
            int nItems = cur_paramlist[i].size;
            if (type == PARAM_TYPE_INT) {
                // parser doesn't handle ints, so convert from doubles here....
                int nAlloc = nItems;
                std::unique_ptr<int[]> idata(new int[nAlloc]);
                double *fdata = (double *)cur_paramlist[i].arg;
                for (int j = 0; j < nAlloc; ++j)
                    idata[j] = int(fdata[j]);
                ps.AddInt(name, std::move(idata), nItems);
            }
            else if (type == PARAM_TYPE_BOOL) {
                // strings -> bools
                int nAlloc = cur_paramlist[i].size;
                std::unique_ptr<bool[]> bdata(new bool[nAlloc]);
                for (int j = 0; j < nAlloc; ++j) {
                    std::string s(((const char **)data)[j]);
                    if (s == "true") bdata[j] = true;
                    else if (s == "false") bdata[j] = false;
                    else {
                        Warning("Value \"%s\" unknown for Boolean parameter \"%s\"."
                            "Using \"false\".", s.c_str(), cur_paramlist[i].name);
                        bdata[j] = false;
                    }
                }
                ps.AddBool(name, std::move(bdata), nItems);
            }
            else if (type == PARAM_TYPE_FLOAT) {
                std::unique_ptr<Float[]> floats(new Float[nItems]);
                for (int i = 0; i < nItems; ++i)
                    floats[i] = ((double *)data)[i];
                ps.AddFloat(name, std::move(floats), nItems);
            } else if (type == PARAM_TYPE_POINT2) {
                if ((nItems % 2) != 0)
                    Warning("Excess values given with point2 parameter \"%s\". "
                            "Ignoring last one of them.", cur_paramlist[i].name);
                std::unique_ptr<Point2f[]> pts(new Point2f[nItems / 2]);
                for (int i = 0; i < nItems / 2; ++i) {
                    pts[i].x = ((double *)data)[2 * i];
                    pts[i].y = ((double *)data)[2 * i + 1];
                }
                ps.AddPoint2f(name, std::move(pts), nItems / 2);
            } else if (type == PARAM_TYPE_VECTOR2) {
                if ((nItems % 2) != 0)
                    Warning("Excess values given with vector2 parameter \"%s\". "
                            "Ignoring last one of them.", cur_paramlist[i].name);
                std::unique_ptr<Vector2f[]> vecs(new Vector2f[nItems / 2]);
                for (int i = 0; i < nItems / 2; ++i) {
                    vecs[i].x = ((double *)data)[2 * i];
                    vecs[i].y = ((double *)data)[2 * i + 1];
                }
                ps.AddVector2f(name, std::move(vecs), nItems / 2);
            } else if (type == PARAM_TYPE_POINT3) {
                if ((nItems % 3) != 0)
                    Warning("Excess values given with point3 parameter \"%s\". "
                            "Ignoring last %d of them.", cur_paramlist[i].name,
                            nItems % 3);
                std::unique_ptr<Point3f[]> pts(new Point3f[nItems / 3]);
                for (int i = 0; i < nItems / 3; ++i) {
                    pts[i].x = ((double *)data)[3 * i];
                    pts[i].y = ((double *)data)[3 * i + 1];
                    pts[i].z = ((double *)data)[3 * i + 2];
                }
                ps.AddPoint3f(name, std::move(pts), nItems / 3);
            } else if (type == PARAM_TYPE_VECTOR3) {
                if ((nItems % 3) != 0)
                    Warning("Excess values given with vector3 parameter \"%s\". "
                            "Ignoring last %d of them.", cur_paramlist[i].name,
                            nItems % 3);
                std::unique_ptr<Vector3f[]> vecs(new Vector3f[nItems / 3]);
                for (int i = 0; i < nItems / 3; ++i) {
                    vecs[i].x = ((double *)data)[3 * i];
                    vecs[i].y = ((double *)data)[3 * i + 1];
                    vecs[i].z = ((double *)data)[3 * i + 2];
                }
                ps.AddVector3f(name, std::move(vecs), nItems / 3);
            } else if (type == PARAM_TYPE_NORMAL) {
                if ((nItems % 3) != 0)
                    Warning("Excess values given with \"normal\" parameter \"%s\". "
                            "Ignoring last %d of them.", cur_paramlist[i].name,
                            nItems % 3);
                std::unique_ptr<Normal3f[]> normals(new Normal3f[nItems / 3]);
                for (int i = 0; i < nItems / 3; ++i) {
                    normals[i].x = ((double *)data)[3 * i];
                    normals[i].y = ((double *)data)[3 * i + 1];
                    normals[i].z = ((double *)data)[3 * i + 2];
                }
                ps.AddNormal3f(name, std::move(normals), nItems / 3);
            } else if (type == PARAM_TYPE_RGB) {
                if ((nItems % 3) != 0)
                    Warning("Excess RGB values given with parameter \"%s\". "
                            "Ignoring last %d of them", cur_paramlist[i].name,
                            nItems % 3);
                std::unique_ptr<Float[]> floats(new Float[nItems]);
                for (int i = 0; i < nItems; ++i)
                    floats[i] = ((double *)data)[i];
                ps.AddRGBSpectrum(name, std::move(floats), nItems);
            } else if (type == PARAM_TYPE_XYZ) {
                if ((nItems % 3) != 0)
                    Warning("Excess XYZ values given with parameter \"%s\". "
                            "Ignoring last %d of them", cur_paramlist[i].name,
                            nItems % 3);
                std::unique_ptr<Float[]> floats(new Float[nItems]);
                for (int i = 0; i < nItems; ++i)
                    floats[i] = ((double *)data)[i];
                ps.AddXYZSpectrum(name, std::move(floats), nItems);
            } else if (type == PARAM_TYPE_BLACKBODY) {
                if ((nItems % 2) != 0)
                    Warning("Excess value given with blackbody parameter \"%s\". "
                            "Ignoring extra one.", cur_paramlist[i].name);
                std::unique_ptr<Float[]> floats(new Float[nItems]);
                for (int i = 0; i < nItems; ++i)
                    floats[i] = ((double *)data)[i];
                ps.AddBlackbodySpectrum(name, std::move(floats), nItems);
            } else if (type == PARAM_TYPE_SPECTRUM) {
                if (cur_paramlist[i].isString) {
                    ps.AddSampledSpectrumFiles(name, (const char **)data, nItems);
                }
                else {
                    if ((nItems % 2) != 0)
                        Warning("Non-even number of values given with sampled spectrum "
                                "parameter \"%s\". Ignoring extra.",
                                cur_paramlist[i].name);
                    std::unique_ptr<Float[]> floats(new Float[nItems]);
                    for (int i = 0; i < nItems; ++i)
                        floats[i] = ((double *)data)[i];
                    ps.AddSampledSpectrum(name, std::move(floats), nItems);
                }
            } else if (type == PARAM_TYPE_STRING) {
                std::unique_ptr<std::string[]> strings(new std::string[nItems]);
                for (int j = 0; j < nItems; ++j)
                    strings[j] = std::string(((const char **)data)[j]);
                ps.AddString(name, std::move(strings), nItems);
            }
            else if (type == PARAM_TYPE_TEXTURE) {
                if (nItems == 1) {
                    std::string val(*((const char **)data));
                    ps.AddTexture(name, val);
                }
                else
                    Error("Only one string allowed for \"texture\" parameter \"%s\"",
                          name.c_str());
            }
        }
        else
            Warning("Type of parameter \"%s\" is unknown", cur_paramlist[i].name);
    }
}


static bool lookupType(const char *name, int *type, std::string &sname) {
    CHECK_NOTNULL(name);
    *type = 0;
    const char *strp = name;
    while (*strp && isspace(*strp))
        ++strp;
    if (!*strp) {
        Error("Parameter \"%s\" doesn't have a type declaration?!", name);
        return false;
    }
#define TRY_DECODING_TYPE(name, mask) \
        if (strncmp(name, strp, strlen(name)) == 0) { \
            *type = mask; strp += strlen(name); \
        }
         TRY_DECODING_TYPE("float",     PARAM_TYPE_FLOAT)
    else TRY_DECODING_TYPE("integer",   PARAM_TYPE_INT)
    else TRY_DECODING_TYPE("bool",      PARAM_TYPE_BOOL)
    else TRY_DECODING_TYPE("point2",    PARAM_TYPE_POINT2)
    else TRY_DECODING_TYPE("vector2",   PARAM_TYPE_VECTOR2)
    else TRY_DECODING_TYPE("point3",    PARAM_TYPE_POINT3)
    else TRY_DECODING_TYPE("vector3",   PARAM_TYPE_VECTOR3)
    else TRY_DECODING_TYPE("point",     PARAM_TYPE_POINT3)
    else TRY_DECODING_TYPE("vector",    PARAM_TYPE_VECTOR3)
    else TRY_DECODING_TYPE("normal",    PARAM_TYPE_NORMAL)
    else TRY_DECODING_TYPE("string",    PARAM_TYPE_STRING)
    else TRY_DECODING_TYPE("texture",   PARAM_TYPE_TEXTURE)
    else TRY_DECODING_TYPE("color",     PARAM_TYPE_RGB)
    else TRY_DECODING_TYPE("rgb",       PARAM_TYPE_RGB)
    else TRY_DECODING_TYPE("xyz",       PARAM_TYPE_XYZ)
    else TRY_DECODING_TYPE("blackbody", PARAM_TYPE_BLACKBODY)
    else TRY_DECODING_TYPE("spectrum",  PARAM_TYPE_SPECTRUM)
    else {
        Error("Unable to decode type for name \"%s\"", name);
        return false;
    }
    while (*strp && isspace(*strp))
        ++strp;
    sname = std::string(strp);
    return true;
}



