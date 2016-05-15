
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


// shapes/plymesh.cpp*
#include "shapes/triangle.h"
#include "textures/constant.h"
#include "paramset.h"
#include "ext/rply.h"

#include <iostream>
using namespace std;

struct CallbackContext {
    Point3f *p;
    Normal3f *n;
    Point2f *uv;
    int *indices;
    int indexCtr;
    int face[4];
    bool error;
    int vertexCount;

    CallbackContext()
        : p(nullptr),
          n(nullptr),
          uv(nullptr),
          indices(nullptr),
          indexCtr(0),
          error(false),
          vertexCount(0) {}

    ~CallbackContext() {
        delete[] p;
        delete[] n;
        delete[] uv;
        delete[] indices;
    }
};

void rply_message_callback(p_ply ply, const char *message) {
    Warning("rply: %s", message);
}

/* Callback to handle vertex data from RPly */
int rply_vertex_callback(p_ply_argument argument) {
    Float **buffers;
    long index, flags;

    ply_get_argument_user_data(argument, (void **)&buffers, &flags);
    ply_get_argument_element(argument, nullptr, &index);

    int bufferIndex = (flags & 0xF00) >> 8;
    int stride = (flags & 0x0F0) >> 4;
    int offset = flags & 0x00F;

    Float *buffer = buffers[bufferIndex];
    if (buffer)
        buffer[index * stride + offset] =
            (float)ply_get_argument_value(argument);

    return 1;
}

/* Callback to handle face data from RPly */
int rply_face_callback(p_ply_argument argument) {
    long length, value_index;
    ply_get_argument_property(argument, nullptr, &length, &value_index);

    if (length != 3 && length != 4) {
        Warning(
            "plymesh: Ignoring face with %i vertices (only triangles and quads "
            "are supported!)",
            (int)length);
        return 1;
    } else if (value_index < 0) {
        return 1;
    }

    CallbackContext *context;
    ply_get_argument_user_data(argument, (void **)&context, nullptr);

    if (value_index >= 0) {
        int value = (int)ply_get_argument_value(argument);
        if (value < 0 || value >= context->vertexCount) {
            Error(
                "plymesh: Vertex reference %i is out of bounds! "
                "Valid range is [0..%i)",
                value, context->vertexCount);
            context->error = true;
        }
        context->face[value_index] = value;
    }

    if (value_index == length - 1) {
        for (int i = 0; i < 3; ++i)
            context->indices[context->indexCtr++] = context->face[i];

        if (length == 4) {
            /* This was a quad */
            context->indices[context->indexCtr++] = context->face[3];
            context->indices[context->indexCtr++] = context->face[0];
            context->indices[context->indexCtr++] = context->face[2];
        }
    }
    return 1;
}

std::vector<std::shared_ptr<Shape>> CreatePLYMesh(
    const Transform *o2w, const Transform *w2o, bool reverseOrientation,
    const ParamSet &params,
    std::map<std::string, std::shared_ptr<Texture<Float>>> *floatTextures) {
    const std::string filename = params.FindOneFilename("filename", "");
    p_ply ply = ply_open(filename.c_str(), rply_message_callback, 0, nullptr);
    if (!ply) {
        Error("Couldn't open PLY file \"%s\"", filename.c_str());
        return std::vector<std::shared_ptr<Shape>>();
    }

    if (!ply_read_header(ply)) {
        Error("Unable to read the header of PLY file \"%s\"", filename.c_str());
        return std::vector<std::shared_ptr<Shape>>();
    }

    p_ply_element element = nullptr;
    long vertexCount = 0, faceCount = 0;

    /* Inspect the structure of the PLY file */
    while ((element = ply_get_next_element(ply, element)) != nullptr) {
        const char *name;
        long nInstances;

        ply_get_element_info(element, &name, &nInstances);
        if (!strcmp(name, "vertex"))
            vertexCount = nInstances;
        else if (!strcmp(name, "face"))
            faceCount = nInstances;
    }

    if (vertexCount == 0 || faceCount == 0) {
        Error("PLY file \"%s\" is invalid! No face/vertex elements found!",
              filename.c_str());
        return std::vector<std::shared_ptr<Shape>>();
    }

    CallbackContext context;

    if (ply_set_read_cb(ply, "vertex", "x", rply_vertex_callback, &context,
                        0x030) &&
        ply_set_read_cb(ply, "vertex", "y", rply_vertex_callback, &context,
                        0x031) &&
        ply_set_read_cb(ply, "vertex", "z", rply_vertex_callback, &context,
                        0x032)) {
        context.p = new Point3f[vertexCount];
    } else {
        Error("PLY file \"%s\": Vertex coordinate property not found!",
              filename.c_str());
        return std::vector<std::shared_ptr<Shape>>();
    }

    if (ply_set_read_cb(ply, "vertex", "nx", rply_vertex_callback, &context,
                        0x130) &&
        ply_set_read_cb(ply, "vertex", "ny", rply_vertex_callback, &context,
                        0x131) &&
        ply_set_read_cb(ply, "vertex", "nz", rply_vertex_callback, &context,
                        0x132))
        context.n = new Normal3f[vertexCount];

    /* There seem to be lots of different conventions regarding UV coordinate
     * names */
    if ((ply_set_read_cb(ply, "vertex", "u", rply_vertex_callback, &context,
                         0x220) &&
         ply_set_read_cb(ply, "vertex", "v", rply_vertex_callback, &context,
                         0x221)) ||
        (ply_set_read_cb(ply, "vertex", "s", rply_vertex_callback, &context,
                         0x220) &&
         ply_set_read_cb(ply, "vertex", "t", rply_vertex_callback, &context,
                         0x221)) ||
        (ply_set_read_cb(ply, "vertex", "texture_u", rply_vertex_callback,
                         &context, 0x220) &&
         ply_set_read_cb(ply, "vertex", "texture_v", rply_vertex_callback,
                         &context, 0x221)) ||
        (ply_set_read_cb(ply, "vertex", "texture_s", rply_vertex_callback,
                         &context, 0x220) &&
         ply_set_read_cb(ply, "vertex", "texture_t", rply_vertex_callback,
                         &context, 0x221)))
        context.uv = new Point2f[vertexCount];

    /* Allocate enough space in case all faces are quads */
    context.indices = new int[faceCount * 6];
    context.vertexCount = vertexCount;

    ply_set_read_cb(ply, "face", "vertex_indices", rply_face_callback, &context,
                    0);

    if (!ply_read(ply)) {
        Error("Unable to read the contents of PLY file \"%s\"",
              filename.c_str());
        ply_close(ply);
        return std::vector<std::shared_ptr<Shape>>();
    }

    ply_close(ply);

    if (context.error) return std::vector<std::shared_ptr<Shape>>();

    // Look up an alpha texture, if applicable
    std::shared_ptr<Texture<Float>> alphaTex;
    std::string alphaTexName = params.FindTexture("alpha");
    if (alphaTexName != "") {
        if (floatTextures->find(alphaTexName) != floatTextures->end())
            alphaTex = (*floatTextures)[alphaTexName];
        else
            Error("Couldn't find float texture \"%s\" for \"alpha\" parameter",
                  alphaTexName.c_str());
    } else if (params.FindOneFloat("alpha", 1.f) == 0.f) {
        alphaTex.reset(new ConstantTexture<Float>(0.f));
    }

    std::shared_ptr<Texture<Float>> shadowAlphaTex;
    std::string shadowAlphaTexName = params.FindTexture("shadowalpha");
    if (shadowAlphaTexName != "") {
        if (floatTextures->find(shadowAlphaTexName) != floatTextures->end())
            shadowAlphaTex = (*floatTextures)[shadowAlphaTexName];
        else
            Error(
                "Couldn't find float texture \"%s\" for \"shadowalpha\" "
                "parameter",
                shadowAlphaTexName.c_str());
    } else if (params.FindOneFloat("shadowalpha", 1.f) == 0.f)
        shadowAlphaTex.reset(new ConstantTexture<Float>(0.f));

    bool discardDegenerateUVs =
        params.FindOneBool("discarddegenerateUVs", false);
    if (discardDegenerateUVs && context.uv && context.n) {
        // if there are normals, check for bad uv's that
        // give degenerate mappings; discard them if so
        const int *vp = context.indices;
        const Point3f *P = context.p;
        const Point2f *uv = context.uv;

        for (int i = 0; i < context.indexCtr; i += 3, vp += 3) {
            Float area =
                .5f * Cross(P[vp[0]] - P[vp[1]], P[vp[2]] - P[vp[1]]).Length();
            if (area < 1e-7) continue;  // ignore degenerate tris.
            if ((uv[vp[0]].x == uv[vp[1]].x && uv[vp[0]].y == uv[vp[1]].y) ||
                (uv[vp[1]].x == uv[vp[2]].x && uv[vp[1]].y == uv[vp[2]].y) ||
                (uv[vp[2]].x == uv[vp[0]].x && uv[vp[2]].y == uv[vp[0]].y)) {
                Warning(
                    "Degenerate uv coordinates in triangle mesh.  Discarding "
                    "all uv.");
                delete[] context.uv;
                context.uv = nullptr;
                break;
            }
        }
    }

    return CreateTriangleMesh(o2w, w2o, reverseOrientation,
                              context.indexCtr / 3, context.indices,
                              vertexCount, context.p, nullptr, context.n,
                              context.uv, alphaTex, shadowAlphaTex);
}
