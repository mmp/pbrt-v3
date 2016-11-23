
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

%{
#include "api.h"
#include "pbrt.h"
#include "paramset.h"
#include <stdarg.h>

#ifdef PBRT_IS_MSVC
#pragma warning(disable:4065)
#pragma warning(disable:4996)
#pragma warning(disable:4018)
#endif // PBRT_IS_MSVC

void yyerror(const char *str) {
    pbrt::Error("Parsing error: %s", str);
    exit(1);
}
extern int yylex();

namespace pbrt {

extern void include_push(char *filename);
int line_num = 0;
std::string current_file;

#define YYMAXDEPTH 100000000

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

}  // namespace pbrt

%}

%union {
char string[1024];
double num;
pbrt::ParamArray *ribarray;
}


%token <string> STRING ID
%token <num> NUM
%token LBRACK RBRACK

%token ACCELERATOR ACTIVETRANSFORM ALL AREALIGHTSOURCE ATTRIBUTEBEGIN
%token ATTRIBUTEEND CAMERA CONCATTRANSFORM COORDINATESYSTEM COORDSYSTRANSFORM
%token ENDTIME FILM IDENTITY INCLUDE LIGHTSOURCE LOOKAT MAKENAMEDMATERIAL
%token MAKENAMEDMEDIUM MATERIAL MEDIUMINTERFACE NAMEDMATERIAL OBJECTBEGIN OBJECTEND
%token OBJECTINSTANCE PIXELFILTER
%token REVERSEORIENTATION ROTATE SAMPLER SCALE SHAPE STARTTIME
%token INTEGRATOR TEXTURE TRANSFORMBEGIN TRANSFORMEND TRANSFORMTIMES
%token TRANSFORM TRANSLATE WORLDBEGIN WORLDEND

%token HIGH_PRECEDENCE

%type<ribarray> array num_array string_array
%%
start: pbrt_stmt_list
{
};



array_init: %prec HIGH_PRECEDENCE
{
    CHECK(pbrt::cur_array == nullptr);
    pbrt::cur_array = new pbrt::ParamArray;
};



string_array_init: %prec HIGH_PRECEDENCE
{
    pbrt::cur_array->element_size = sizeof(const char *);
    pbrt::cur_array->isString = true;
};



num_array_init: %prec HIGH_PRECEDENCE
{
    pbrt::cur_array->element_size = sizeof(double);
    pbrt::cur_array->isString = false;
};



array: string_array
{
    $$ = $1;
}


| num_array
{
    $$ = $1;
};



string_array: array_init LBRACK string_list RBRACK
{
    $$ = pbrt::cur_array;
    pbrt::cur_array = nullptr;
}


| single_element_string_array
{
    $$ = pbrt::cur_array;
    pbrt::cur_array = nullptr;
};



single_element_string_array: array_init string_list_entry
{
};



string_list: string_list string_list_entry
{
}


| string_list_entry
{
};



string_list_entry: string_array_init STRING
{
    char *to_add = strdup($2);
    pbrt::AddArrayElement(&to_add);
};



num_array: array_init LBRACK num_list RBRACK
{
    $$ = pbrt::cur_array;
    pbrt::cur_array = nullptr;
}


| single_element_num_array
{
    $$ = pbrt::cur_array;
    pbrt::cur_array = nullptr;
};



single_element_num_array: array_init num_list_entry
{
};



num_list: num_list num_list_entry
{
}


| num_list_entry
{
};



num_list_entry: num_array_init NUM
{
    double to_add = $2;
    pbrt::AddArrayElement(&to_add);
};



paramlist: paramlist_init paramlist_contents
{
};



paramlist_init: %prec HIGH_PRECEDENCE
{
    for (size_t i = 0; i < pbrt::cur_paramlist.size(); ++i) {
        if (pbrt::cur_paramlist[i].isString) {
            for (size_t j = 0; j < pbrt::cur_paramlist[i].size; ++j)
                free(((char **)pbrt::cur_paramlist[i].arg)[j]);
        }
    }
    pbrt::cur_paramlist.erase(pbrt::cur_paramlist.begin(), pbrt::cur_paramlist.end());
};



paramlist_contents: paramlist_entry paramlist_contents
{
}


|
{
};



paramlist_entry: STRING array
{
    pbrt::cur_paramlist.push_back(pbrt::ParamListItem($1, $2));
    pbrt::ArrayFree($2);
};



pbrt_stmt_list: pbrt_stmt_list pbrt_stmt
{
}


| pbrt_stmt
{
};



pbrt_stmt: ACCELERATOR STRING paramlist
{
    pbrt::ParamSet params;
    pbrt::InitParamSet(params, pbrt::SpectrumType::Reflectance);
    pbrt::pbrtAccelerator($2, params);
    pbrt::FreeArgs();
}


| ACTIVETRANSFORM ALL
{
    pbrt::pbrtActiveTransformAll();
}


| ACTIVETRANSFORM ENDTIME
{
    pbrt::pbrtActiveTransformEndTime();
}


| ACTIVETRANSFORM STARTTIME
{
    pbrt::pbrtActiveTransformStartTime();
}


| AREALIGHTSOURCE STRING paramlist
{
    pbrt::ParamSet params;
    pbrt::InitParamSet(params, pbrt::SpectrumType::Illuminant);
    pbrt::pbrtAreaLightSource($2, params);
    pbrt::FreeArgs();
}


| ATTRIBUTEBEGIN
{
    pbrt::pbrtAttributeBegin();
}


| ATTRIBUTEEND
{
    pbrt::pbrtAttributeEnd();
}


| CAMERA STRING paramlist
{
    pbrt::ParamSet params;
    pbrt::InitParamSet(params, pbrt::SpectrumType::Reflectance);
    pbrt::pbrtCamera($2, params);
    pbrt::FreeArgs();
}


| CONCATTRANSFORM num_array
{
    if (pbrt::VerifyArrayLength($2, 16, "ConcatTransform")) {
        pbrt::Float m[16];
        double *dm = (double *)$2->array;
        std::copy(dm, dm + 16, m);
        pbrt::pbrtConcatTransform(m);
    }
    pbrt::ArrayFree($2);
}


| COORDINATESYSTEM STRING
{
    pbrt::pbrtCoordinateSystem($2);
}


| COORDSYSTRANSFORM STRING
{
    pbrt::pbrtCoordSysTransform($2);
}


| FILM STRING paramlist
{
    pbrt::ParamSet params;
    pbrt::InitParamSet(params, pbrt::SpectrumType::Reflectance);
    pbrt::pbrtFilm($2, params);
    pbrt::FreeArgs();
}


| IDENTITY
{
    pbrt::pbrtIdentity();
}


| INCLUDE STRING
{
    pbrt::include_push($2);
}


| LIGHTSOURCE STRING paramlist
{
    pbrt::ParamSet params;
    pbrt::InitParamSet(params, pbrt::SpectrumType::Illuminant);
    pbrt::pbrtLightSource($2, params);
    pbrt::FreeArgs();
}


| LOOKAT NUM NUM NUM NUM NUM NUM NUM NUM NUM
{
    pbrt::pbrtLookAt($2, $3, $4, $5, $6, $7, $8, $9, $10);
}


| MAKENAMEDMATERIAL STRING paramlist
{
    pbrt::ParamSet params;
    pbrt::InitParamSet(params, pbrt::SpectrumType::Reflectance);
    pbrt::pbrtMakeNamedMaterial($2, params);
    pbrt::FreeArgs();
}


| MAKENAMEDMEDIUM STRING paramlist
{
    pbrt::ParamSet params;
    pbrt::InitParamSet(params, pbrt::SpectrumType::Reflectance);
    pbrt::pbrtMakeNamedMedium($2, params);
    pbrt::FreeArgs();
}


| MATERIAL STRING paramlist
{
    pbrt::ParamSet params;
    pbrt::InitParamSet(params, pbrt::SpectrumType::Reflectance);
    pbrt::pbrtMaterial($2, params);
    pbrt::FreeArgs();
}


| MEDIUMINTERFACE STRING
{
    pbrt::pbrtMediumInterface($2, $2);
}


| MEDIUMINTERFACE STRING STRING
{
    pbrt::pbrtMediumInterface($2, $3);
}


| NAMEDMATERIAL STRING
{
    pbrt::pbrtNamedMaterial($2);
}


| OBJECTBEGIN STRING
{
    pbrt::pbrtObjectBegin($2);
}


| OBJECTEND
{
    pbrt::pbrtObjectEnd();
}


| OBJECTINSTANCE STRING
{
    pbrt::pbrtObjectInstance($2);
}


| PIXELFILTER STRING paramlist
{
    pbrt::ParamSet params;
    pbrt::InitParamSet(params, pbrt::SpectrumType::Reflectance);
    pbrt::pbrtPixelFilter($2, params);
    pbrt::FreeArgs();
}


| REVERSEORIENTATION
{
    pbrt::pbrtReverseOrientation();
}


| ROTATE NUM NUM NUM NUM
{
    pbrt::pbrtRotate($2, $3, $4, $5);
}


| SAMPLER STRING paramlist
{
    pbrt::ParamSet params;
    pbrt::InitParamSet(params, pbrt::SpectrumType::Reflectance);
    pbrt::pbrtSampler($2, params);
    pbrt::FreeArgs();
}


| SCALE NUM NUM NUM
{
    pbrt::pbrtScale($2, $3, $4);
}


| SHAPE STRING paramlist
{
    pbrt::ParamSet params;
    pbrt::InitParamSet(params, pbrt::SpectrumType::Reflectance);
    pbrt::pbrtShape($2, params);
    pbrt::FreeArgs();
}


| INTEGRATOR STRING paramlist
{
    pbrt::ParamSet params;
    pbrt::InitParamSet(params, pbrt::SpectrumType::Reflectance);
    pbrt::pbrtIntegrator($2, params);
    pbrt::FreeArgs();
}


| TEXTURE STRING STRING STRING paramlist
{
    pbrt::ParamSet params;
    pbrt::InitParamSet(params, pbrt::SpectrumType::Reflectance);
    pbrt::pbrtTexture($2, $3, $4, params);
    pbrt::FreeArgs();
}


| TRANSFORMBEGIN
{
    pbrt::pbrtTransformBegin();
}


| TRANSFORMEND
{
    pbrt::pbrtTransformEnd();
}


| TRANSFORMTIMES NUM NUM
{
    pbrt::pbrtTransformTimes($2, $3);
}


| TRANSFORM num_array
{
    if (pbrt::VerifyArrayLength($2, 16, "Transform")) {
        pbrt::Float m[16];
        double *dm = (double *)$2->array;
        std::copy(dm, dm + 16, m);
        pbrt::pbrtTransform(m);
    }
    pbrt::ArrayFree($2);
}


| TRANSLATE NUM NUM NUM
{
    pbrt::pbrtTranslate($2, $3, $4);
}


| WORLDBEGIN
{
    pbrt::pbrtWorldBegin();
}


| WORLDEND
{
    pbrt::pbrtWorldEnd();
};


%%

namespace pbrt {

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
                for (int j = 0; j < nItems / 3; ++j) {
                    vecs[j].x = ((double *)data)[3 * j];
                    vecs[j].y = ((double *)data)[3 * j + 1];
                    vecs[j].z = ((double *)data)[3 * j + 2];
                }
                ps.AddVector3f(name, std::move(vecs), nItems / 3);
            } else if (type == PARAM_TYPE_NORMAL) {
                if ((nItems % 3) != 0)
                    Warning("Excess values given with \"normal\" parameter \"%s\". "
                            "Ignoring last %d of them.", cur_paramlist[i].name,
                            nItems % 3);
                std::unique_ptr<Normal3f[]> normals(new Normal3f[nItems / 3]);
                for (int j = 0; j < nItems / 3; ++j) {
                    normals[j].x = ((double *)data)[3 * j];
                    normals[j].y = ((double *)data)[3 * j + 1];
                    normals[j].z = ((double *)data)[3 * j + 2];
                }
                ps.AddNormal3f(name, std::move(normals), nItems / 3);
            } else if (type == PARAM_TYPE_RGB) {
                if ((nItems % 3) != 0) {
                    Warning("Excess RGB values given with parameter \"%s\". "
                            "Ignoring last %d of them", cur_paramlist[i].name,
                            nItems % 3);
                    nItems -= nItems % 3;
                }
                std::unique_ptr<Float[]> floats(new Float[nItems]);
                for (int j = 0; j < nItems; ++j)
                    floats[j] = ((double *)data)[j];
                ps.AddRGBSpectrum(name, std::move(floats), nItems);
            } else if (type == PARAM_TYPE_XYZ) {
                if ((nItems % 3) != 0) {
                    Warning("Excess XYZ values given with parameter \"%s\". "
                            "Ignoring last %d of them", cur_paramlist[i].name,
                            nItems % 3);
                    nItems -= nItems % 3;
                }
                std::unique_ptr<Float[]> floats(new Float[nItems]);
                for (int j = 0; j < nItems; ++j)
                    floats[j] = ((double *)data)[j];
                ps.AddXYZSpectrum(name, std::move(floats), nItems);
            } else if (type == PARAM_TYPE_BLACKBODY) {
                if ((nItems % 2) != 0) {
                    Warning("Excess value given with blackbody parameter \"%s\". "
                            "Ignoring extra one.", cur_paramlist[i].name);
                    nItems -= nItems % 2;
                }
                std::unique_ptr<Float[]> floats(new Float[nItems]);
                for (int j = 0; j < nItems; ++j)
                    floats[j] = ((double *)data)[j];
                ps.AddBlackbodySpectrum(name, std::move(floats), nItems);
            } else if (type == PARAM_TYPE_SPECTRUM) {
                if (cur_paramlist[i].isString) {
                    ps.AddSampledSpectrumFiles(name, (const char **)data, nItems);
                }
                else {
                    if ((nItems % 2) != 0) {
                        Warning("Non-even number of values given with sampled spectrum "
                                "parameter \"%s\". Ignoring extra.",
                                cur_paramlist[i].name);
                        nItems -= nItems % 2;
                    }
                    std::unique_ptr<Float[]> floats(new Float[nItems]);
                    for (int j = 0; j < nItems; ++j)
                        floats[j] = ((double *)data)[j];
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

}  // namespace pbrt
