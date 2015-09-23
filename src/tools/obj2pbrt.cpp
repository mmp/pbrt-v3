//
// obj2pbrt.cpp
//
// Convert Wavefront OBJ files to PBRT.
// Based on Syoyo Fujita's tinyobjloader:
//    https://github.com/syoyo/tinyobjloader
//
// 2-clause BSD license
//

///////////// start of tiny_obj_loader.h

//
// Copyright 2012-2013, Syoyo Fujita.
//
// Licensed under 2-clause BSD liecense.
//
#ifndef _TINY_OBJ_LOADER_H
#define _TINY_OBJ_LOADER_H

#include <string>
#include <vector>
#include <map>

namespace tinyobj {

typedef struct
{
    std::string name;

    float ambient[3];
    float diffuse[3];
    float specular[3];
    float transmittance[3];
    float emission[3];
    float shininess;
    float ior;                // index of refraction
    float dissolve;           // 1 == opaque; 0 == fully transparent
    // illumination model (see http://www.fileformat.info/format/material/)
    int illum;
    float bump_scale;

    std::string ambient_texname;
    std::string diffuse_texname;
    std::string specular_texname;
    std::string normal_texname;
    std::map<std::string, std::string> unknown_parameter;
} material_t;

typedef struct
{
    std::vector<float>          positions;
    std::vector<float>          normals;
    std::vector<float>          texcoords;
    std::vector<unsigned int>   indices;
} mesh_t;

typedef struct
{
    std::string  name;
    material_t   material;
    mesh_t       mesh;
} shape_t;

/// Loads .obj from a file.
/// 'shapes' will be filled with parsed shape data
/// The function returns error string.
/// Returns empty string when loading .obj success.
/// 'mtl_basepath' is optional, and used for base path for .mtl file.
std::string LoadObj(
    std::vector<shape_t>& shapes,   // [output]
    const char* filename,
    const char* mtl_basepath = NULL);

};

#endif  // _TINY_OBJ_LOADER_H


///////////// start of tiny_obj_loader.cc

//
// Copyright 2012-2013, Syoyo Fujita.
// 
// Licensed under 2-clause BSD liecense.
//

//
// version 0.9.6: Support Ni(index of refraction) mtl parameter.
//                Parse transmittance material parameter correctly.
// version 0.9.5: Parse multiple group name.
//                Add support of specifying the base path to load material file.
// version 0.9.4: Initial suupport of group tag(g)
// version 0.9.3: Fix parsing triple 'x/y/z'
// version 0.9.2: Add more .mtl load support
// version 0.9.1: Add initial .mtl load support
// version 0.9.0: Initial
//


#include <cstdlib>
#include <cstring>
#include <cassert>

#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>

namespace tinyobj {

struct vertex_index {
  int v_idx, vt_idx, vn_idx;
  vertex_index() {};
  vertex_index(int idx) : v_idx(idx), vt_idx(idx), vn_idx(idx) {};
  vertex_index(int vidx, int vtidx, int vnidx) : v_idx(vidx), vt_idx(vtidx), vn_idx(vnidx) {};

};
// for std::map
static inline bool operator<(const vertex_index& a, const vertex_index& b)
{
  if (a.v_idx != b.v_idx) return (a.v_idx < b.v_idx);
  if (a.vn_idx != b.vn_idx) return (a.vn_idx < b.vn_idx);
  if (a.vt_idx != b.vt_idx) return (a.vt_idx < b.vt_idx);

  return false;
}

struct obj_shape {
  std::vector<float> v;
  std::vector<float> vn;
  std::vector<float> vt;
};

static inline bool isSpace(const char c) {
  return (c == ' ') || (c == '\t');
}

static inline bool isNewLine(const char c) {
  return (c == '\r') || (c == '\n') || (c == '\0');
}

// Make index zero-base, and also support relative index. 
static inline int fixIndex(int idx, int n)
{
  int i;

  if (idx > 0) {
    i = idx - 1;
  } else if (idx == 0) {
    i = 0;
  } else { // negative value = relative
    i = n + idx;
  }
  return i;
}

static inline std::string parseString(const char*& token)
{
  std::string s;
  int b = strspn(token, " \t");
  int e = strcspn(token, " \t\r");
  s = std::string(&token[b], &token[e]);

  token += (e - b);
  return s;
}

static inline int parseInt(const char*& token)
{
  token += strspn(token, " \t");
  int i = atoi(token);
  token += strcspn(token, " \t\r");
  return i;
}

static inline float parseFloat(const char*& token)
{
  token += strspn(token, " \t");
  float f = (float)atof(token);
  token += strcspn(token, " \t\r");
  return f;
}

static inline void parseFloat2(
  float& x, float& y,
  const char*& token)
{
  x = parseFloat(token);
  y = parseFloat(token);
}

static inline void parseFloat3(
  float& x, float& y, float& z,
  const char*& token)
{
  x = parseFloat(token);
  y = parseFloat(token);
  z = parseFloat(token);
}


// Parse triples: i, i/j/k, i//k, i/j
static vertex_index parseTriple(
  const char* &token,
  int vsize,
  int vnsize,
  int vtsize)
{
    vertex_index vi(-1);

    vi.v_idx = fixIndex(atoi(token), vsize);
    token += strcspn(token, "/ \t\r");
    if (token[0] != '/') {
      return vi;
    }
    token++;

    // i//k
    if (token[0] == '/') {
      token++;
      vi.vn_idx = fixIndex(atoi(token), vnsize);
      token += strcspn(token, "/ \t\r");
      return vi;
    }
    
    // i/j/k or i/j
    vi.vt_idx = fixIndex(atoi(token), vtsize);
    token += strcspn(token, "/ \t\r");
    if (token[0] != '/') {
      return vi;
    }

    // i/j/k
    token++;  // skip '/'
    vi.vn_idx = fixIndex(atoi(token), vnsize);
    token += strcspn(token, "/ \t\r");
    return vi; 
}

static unsigned int
updateVertex(
  std::map<vertex_index, unsigned int>& vertexCache,
  std::vector<float>& positions,
  std::vector<float>& normals,
  std::vector<float>& texcoords,
  const std::vector<float>& in_positions,
  const std::vector<float>& in_normals,
  const std::vector<float>& in_texcoords,
  const vertex_index& i)
{
  const std::map<vertex_index, unsigned int>::iterator it = vertexCache.find(i);

  if (it != vertexCache.end()) {
    // found cache
    return it->second;
  }

  assert(in_positions.size() > (3*i.v_idx+2));

  positions.push_back(in_positions[3*i.v_idx+0]);
  positions.push_back(in_positions[3*i.v_idx+1]);
  positions.push_back(in_positions[3*i.v_idx+2]);

  if (i.vn_idx >= 0) {
    normals.push_back(in_normals[3*i.vn_idx+0]);
    normals.push_back(in_normals[3*i.vn_idx+1]);
    normals.push_back(in_normals[3*i.vn_idx+2]);
  }

  if (i.vt_idx >= 0) {
    texcoords.push_back(in_texcoords[2*i.vt_idx+0]);
    texcoords.push_back(in_texcoords[2*i.vt_idx+1]);
  }

  unsigned int idx = positions.size() / 3 - 1;
  vertexCache[i] = idx;

  return idx;
}

static bool
exportFaceGroupToShape(
  shape_t& shape,
  const std::vector<float> &in_positions,
  const std::vector<float> &in_normals,
  const std::vector<float> &in_texcoords,
  const std::vector<std::vector<vertex_index> >& faceGroup,
  const material_t &material,
  const std::string &name)
{
  if (faceGroup.empty()) {
    return false;
  }

  // Flattened version of vertex data
  std::vector<float> positions;
  std::vector<float> normals;
  std::vector<float> texcoords;
  std::map<vertex_index, unsigned int> vertexCache;
  std::vector<unsigned int> indices;

  // Flatten vertices and indices
  for (size_t i = 0; i < faceGroup.size(); i++) {
    const std::vector<vertex_index>& face = faceGroup[i];

    vertex_index i0 = face[0];
    vertex_index i1(-1);
    vertex_index i2 = face[1];

    size_t npolys = face.size();

    // Polygon -> triangle fan conversion
    for (size_t k = 2; k < npolys; k++) {
      i1 = i2;
      i2 = face[k];

      unsigned int v0 = updateVertex(vertexCache, positions, normals, texcoords, in_positions, in_normals, in_texcoords, i0);
      unsigned int v1 = updateVertex(vertexCache, positions, normals, texcoords, in_positions, in_normals, in_texcoords, i1);
      unsigned int v2 = updateVertex(vertexCache, positions, normals, texcoords, in_positions, in_normals, in_texcoords, i2);

      indices.push_back(v0);
      indices.push_back(v1);
      indices.push_back(v2);
    }

  }

  //
  // Construct shape.
  //
  shape.name = name;
  shape.mesh.positions.swap(positions);
  shape.mesh.normals.swap(normals);
  shape.mesh.texcoords.swap(texcoords);
  shape.mesh.indices.swap(indices);

  shape.material = material;

  return true;

}

bool IsBumpMap(const char *&token) {
    if ((0 == strncmp(token, "map_Ns", 6)) && isSpace(token[6])) {
      token += 7;
      return true;
    }
    if ((0 == strncmp(token, "bump", 4)) && isSpace(token[4])) {
      token += 5;
      return true;
    }
    if ((0 == strncmp(token, "map_bump", 8)) && isSpace(token[8])) {
      token += 9;
      return true;
    }
    if ((0 == strncmp(token, "map_Bump", 8)) && isSpace(token[8])) {
      token += 9;
      return true;
    }
    return false;
}

bool IsFloat(const char *token) {
  if (isdigit(*token))
    return true;
  else if (*token == '-')
    return isdigit(token[1]) || (token[1] == '.' && isdigit(token[2]));
  else if (*token == '.')
    return isdigit(token[1]);
  else
    return false;
}

void ParseUnsupportedBumpModifier(const char *&token, std::stringstream &err) {
  if ((0 == strncmp(token, "-mm", 3)) && isSpace(token[3])) {
    token += 4;
    (void)parseFloat(token);  // base_value
    (void)parseFloat(token);  // gain_value
    err << "Ignoring bump map modifier \"-mm\"" << std::endl;
  }
  else if ((0 == strncmp(token, "-blendu", 7)) && isSpace(token[7])) {
    token += 8;
    (void)parseString(token);
    err << "Ignoring bump map modifier \"-blendu\"" << std::endl;
  }
  else if ((0 == strncmp(token, "-blendv", 7)) && isSpace(token[7])) {
    token += 8;
    (void)parseString(token);
    err << "Ignoring bump map modifier \"-blendv\"" << std::endl;
  }
  else if ((0 == strncmp(token, "-boost", 6)) && isSpace(token[6])) {
    token += 7;
    (void)parseFloat(token);
    err << "Ignoring bump map modifier \"-boost\"" << std::endl;
  }
  else if ((0 == strncmp(token, "-texres", 7)) && isSpace(token[7])) {
    token += 8;
    (void)parseFloat(token);
    err << "Ignoring bump map modifier \"-texres\"" << std::endl;
  }
  else if ((0 == strncmp(token, "-clamp", 6)) && isSpace(token[6])) {
    token += 7;
    (void)parseString(token);
    err << "Ignoring bump map modifier \"-clamp\"" << std::endl;
  }
  else if ((0 == strncmp(token, "-imfchan", 8)) && isSpace(token[8])) {
    token += 9;
    (void)parseString(token);
    err << "Ignoring bump map modifier \"-imfchan\"" << std::endl;
  }
  else if ((0 == strncmp(token, "-type", 5)) && isSpace(token[5])) {
    token += 6;
    (void)parseString(token);
    err << "Ignoring bump map modifier \"-type\"" << std::endl;
  }
  else if ((0 == strncmp(token, "-o", 2)) && isSpace(token[2])) {
    token += 3;
    (void)parseFloat(token);  // x
    if (IsFloat(token)) {
      (void)parseFloat(token);  // y
      if (IsFloat(token)) {
        (void)parseFloat(token);  // z
      }
    }
    err << "Ignoring bump map modifier \"-o\"" << std::endl;
  }
  else if ((0 == strncmp(token, "-s", 2)) && isSpace(token[2])) {
    token += 3;
    (void)parseFloat(token);  // x
    if (IsFloat(token)) {
      (void)parseFloat(token);  // y
      if (IsFloat(token)) {
        (void)parseFloat(token);  // z
      }
    }
    err << "Ignoring bump map modifier \"-s\"" << std::endl;
  }
  else if ((0 == strncmp(token, "-t", 2)) && isSpace(token[2])) {
    token += 3;
    (void)parseFloat(token);  // x
    if (IsFloat(token)) {
      (void)parseFloat(token);  // y
      if (IsFloat(token)) {
        (void)parseFloat(token);  // z
      }
    }
    err << "Ignoring bump map modifier \"-t\"" << std::endl;
  }
  else
    err << "Unexpected modifier \"" << token << "\" for bump map" << std::endl;
}

void InitMaterial(material_t& material) {
  material.name = "";
  material.ambient_texname = "";
  material.diffuse_texname = "";
  material.specular_texname = "";
  material.normal_texname = "";
  for (int i = 0; i < 3; i ++) {
    material.ambient[i] = 0.f;
    material.diffuse[i] = 0.f;
    material.specular[i] = 0.f;
    material.transmittance[i] = 0.f;
    material.emission[i] = 0.f;
  }
  material.illum = 0;
  material.dissolve = 1.f;
  material.shininess = 1.f;
  material.ior = 1.f;
  material.bump_scale = 1.f;
  material.unknown_parameter.clear();
}

std::string LoadMtl (
  std::map<std::string, material_t>& material_map,
  const char* filename,
  const char* mtl_basepath)
{
  material_map.clear();
  std::stringstream err;

  std::string filepath;

  if (mtl_basepath) {
    filepath = std::string(mtl_basepath) + std::string(filename);
  } else {
    filepath = std::string(filename);
  }

  std::ifstream ifs(filepath.c_str());
  if (!ifs) {
    err << "Cannot open file [" << filepath << "]" << std::endl;
    return err.str();
  }

  material_t material;
  
  int maxchars = 8192;  // Alloc enough size.
  std::vector<char> buf(maxchars);  // Alloc enough size.
  while (ifs.peek() != -1) {
    ifs.getline(&buf[0], maxchars);

    std::string linebuf(&buf[0]);

    // Trim newline '\r\n' or '\r'
    if (linebuf.size() > 0) {
      if (linebuf[linebuf.size()-1] == '\n') linebuf.erase(linebuf.size()-1);
    }
    if (linebuf.size() > 0) {
      if (linebuf[linebuf.size()-1] == '\n') linebuf.erase(linebuf.size()-1);
    }

    // Skip if empty line.
    if (linebuf.empty()) {
      continue;
    }

    // Skip leading space.
    const char* token = linebuf.c_str();
    token += strspn(token, " \t");

    assert(token);
    if (token[0] == '\0') continue; // empty line
    
    if (token[0] == '#') continue;  // comment line
    
    // new mtl
    if ((0 == strncmp(token, "newmtl", 6)) && isSpace((token[6]))) {
      // flush previous material.
      material_map.insert(std::pair<std::string, material_t>(material.name, material));

      // initial temporary material
      InitMaterial(material);

      // set new mtl name
      char namebuf[4096];
      token += 7;
      sscanf(token, "%s", namebuf);
      material.name = namebuf;
      continue;
    }
    
    // ambient
    if (token[0] == 'K' && token[1] == 'a' && isSpace((token[2]))) {
      token += 2;
      float r, g, b;
      parseFloat3(r, g, b, token);
      material.ambient[0] = r;
      material.ambient[1] = g;
      material.ambient[2] = b;
      continue;
    }
    
    // diffuse
    if (token[0] == 'K' && token[1] == 'd' && isSpace((token[2]))) {
      token += 2;
      float r, g, b;
      parseFloat3(r, g, b, token);
      material.diffuse[0] = r;
      material.diffuse[1] = g;
      material.diffuse[2] = b;
      continue;
    }
    
    // specular
    if (token[0] == 'K' && token[1] == 's' && isSpace((token[2]))) {
      token += 2;
      float r, g, b;
      parseFloat3(r, g, b, token);
      material.specular[0] = r;
      material.specular[1] = g;
      material.specular[2] = b;
      continue;
    }
    
    // transmittance
    if (token[0] == 'K' && token[1] == 't' && isSpace((token[2]))) {
      token += 2;
      float r, g, b;
      parseFloat3(r, g, b, token);
      material.transmittance[0] = r;
      material.transmittance[1] = g;
      material.transmittance[2] = b;
      continue;
    }

    // ior(index of refraction)
    if (token[0] == 'N' && token[1] == 'i' && isSpace((token[2]))) {
      token += 2;
      material.ior = parseFloat(token);
      continue;
    }

    // emission
    if(token[0] == 'K' && token[1] == 'e' && isSpace(token[2])) {
      token += 2;
      float r, g, b;
      parseFloat3(r, g, b, token);
      material.emission[0] = r;
      material.emission[1] = g;
      material.emission[2] = b;
      continue;
    }

    // shininess
    if(token[0] == 'N' && token[1] == 's' && isSpace(token[2])) {
      token += 2;
      material.shininess = parseFloat(token);
      continue;
    }

    // illum model
    if (0 == strncmp(token, "illum", 5) && isSpace(token[5])) {
      token += 6;
      material.illum = parseInt(token);
      continue;
    }

    // dissolve
    if ((token[0] == 'd' && isSpace(token[1]))) {
      token += 1;
      material.dissolve = parseFloat(token);
      continue;
    }
    if (token[0] == 'T' && token[1] == 'r' && isSpace(token[2])) {
      token += 2;
      material.dissolve = parseFloat(token);
      continue;
    }

    // ambient texture
    if ((0 == strncmp(token, "map_Ka", 6)) && isSpace(token[6])) {
      token += 7;
      material.ambient_texname = token;
      continue;
    }

    // diffuse texture
    if ((0 == strncmp(token, "map_Kd", 6)) && isSpace(token[6])) {
      token += 7;
      material.diffuse_texname = token;
      continue;
    }

    // specular texture
    if ((0 == strncmp(token, "map_Ks", 6)) && isSpace(token[6])) {
      token += 7;
      material.specular_texname = token;
      continue;
    }

    // normal texture
    if (IsBumpMap(token)) {
      while (*token == '-') {
        // parse bump map options...
        if((0 == strncmp(token, "-bm", 3)) && isSpace(token[3])) {
          token += 4;
          material.bump_scale = parseFloat(token);
        }
        else {
          ParseUnsupportedBumpModifier(token, err);
        }
        // skip space
        token += strspn(token, " \t");
      }
      material.normal_texname = token;
      continue;
    }

    // unknown parameter
    const char* _space = strchr(token, ' ');
    if(!_space) {
      _space = strchr(token, '\t');
    }
    if(_space) {
      int len = _space - token;
      std::string key(token, len);
      std::string value = _space + 1;
      material.unknown_parameter.insert(std::pair<std::string, std::string>(key, value));
    }
  }
  // flush last material.
  material_map.insert(std::pair<std::string, material_t>(material.name, material));

  return err.str();
}

std::string
LoadObj(
  std::vector<shape_t>& shapes,
  const char* filename,
  const char* mtl_basepath)
{

  shapes.clear();

  std::stringstream err;

  std::ifstream ifs(filename);
  if (!ifs) {
    err << "Cannot open file [" << filename << "]" << std::endl;
    return err.str();
  }

  std::vector<float> v;
  std::vector<float> vn;
  std::vector<float> vt;
  std::vector<std::vector<vertex_index> > faceGroup;
  std::string name;

  // material
  std::map<std::string, material_t> material_map;
  material_t material;

  int maxchars = 8192;  // Alloc enough size.
  std::vector<char> buf(maxchars);  // Alloc enough size.
  while (ifs.peek() != -1) {
    ifs.getline(&buf[0], maxchars);

    std::string linebuf(&buf[0]);

    // Trim newline '\r\n' or '\r'
    if (linebuf.size() > 0) {
      if (linebuf[linebuf.size()-1] == '\n') linebuf.erase(linebuf.size()-1);
    }
    if (linebuf.size() > 0) {
      if (linebuf[linebuf.size()-1] == '\n') linebuf.erase(linebuf.size()-1);
    }

    // Skip if empty line.
    if (linebuf.empty()) {
      continue;
    }

    // Skip leading space.
    const char* token = linebuf.c_str();
    token += strspn(token, " \t");

    assert(token);
    if (token[0] == '\0') continue; // empty line
    
    if (token[0] == '#') continue;  // comment line

    // vertex
    if (token[0] == 'v' && isSpace((token[1]))) {
      token += 2;
      float x, y, z;
      parseFloat3(x, y, z, token);
      v.push_back(x);
      v.push_back(y);
      v.push_back(z);
      continue;
    }

    // normal
    if (token[0] == 'v' && token[1] == 'n' && isSpace((token[2]))) {
      token += 3;
      float x, y, z;
      parseFloat3(x, y, z, token);
      vn.push_back(x);
      vn.push_back(y);
      vn.push_back(z);
      continue;
    }

    // texcoord
    if (token[0] == 'v' && token[1] == 't' && isSpace((token[2]))) {
      token += 3;
      float x, y;
      parseFloat2(x, y, token);
      vt.push_back(x);
      vt.push_back(y);
      continue;
    }

    // face
    if (token[0] == 'f' && isSpace((token[1]))) {
      token += 2;
      token += strspn(token, " \t");

      std::vector<vertex_index> face;
      while (!isNewLine(token[0])) {
        vertex_index vi = parseTriple(token, v.size() / 3, vn.size() / 3, vt.size() / 2);
        face.push_back(vi);
        int n = strspn(token, " \t\r");
        token += n;
      }

      faceGroup.push_back(face);
      
      continue;
    }

    // use mtl
    if ((0 == strncmp(token, "usemtl", 6)) && isSpace((token[6]))) {

      char namebuf[4096];
      token += 7;
      sscanf(token, "%s", namebuf);

      if (material_map.find(namebuf) != material_map.end()) {
        material = material_map[namebuf];
      } else {
        // { error!! material not found }
        InitMaterial(material);
      }
      continue;

    }

    // load mtl
    if ((0 == strncmp(token, "mtllib", 6)) && isSpace((token[6]))) {
      char namebuf[4096];
      token += 7;
      sscanf(token, "%s", namebuf);

      std::string err_mtl = LoadMtl(material_map, namebuf, mtl_basepath);
      if (!err_mtl.empty()) {
        faceGroup.clear();  // for safety
        return err_mtl;
      }
      continue;
    }

    // group name
    if (token[0] == 'g' && isSpace((token[1]))) {

      // flush previous face group.
      shape_t shape;
      bool ret = exportFaceGroupToShape(shape, v, vn, vt, faceGroup, material, name);
      if (ret) {
        shapes.push_back(shape);
      }

      faceGroup.clear();

      std::vector<std::string> names;
      while (!isNewLine(token[0])) {
        std::string str = parseString(token);
        names.push_back(str);
        token += strspn(token, " \t\r"); // skip tag
      }

      assert(names.size() > 0);

      // names[0] must be 'g', so skipt 0th element.
      if (names.size() > 1) {
        name = names[1];
      } else {
        name = "";
      }

      continue;
    }

    // object name
    if (token[0] == 'o' && isSpace((token[1]))) {

      // flush previous face group.
      shape_t shape;
      bool ret = exportFaceGroupToShape(shape, v, vn, vt, faceGroup, material, name);
      if (ret) {
        shapes.push_back(shape);
      }

      faceGroup.clear();

      // @todo { multiple object name? }
      char namebuf[4096];
      token += 2;
      sscanf(token, "%s", namebuf);
      name = std::string(namebuf);


      continue;
    }

    // Ignore unknown command.
  }

  shape_t shape;
  bool ret = exportFaceGroupToShape(shape, v, vn, vt, faceGroup, material, name);
  if (ret) {
    shapes.push_back(shape);
  }
  faceGroup.clear();  // for safety

  return err.str();
}


};


///////////////////////////////////////////////////////////////////////////
// The above is tiny_obj_loader.{h,cc} basically directly; pbrt specific
// code follows...

using namespace tinyobj;

int main(int argc, char *argv[]) {
  if (argc != 3 || strcmp(argv[1], "--help") == 0 ||
      strcmp(argv[1], "-h") == 0) {
    fprintf(stderr, "usage: obj2pbrt [OBJ filename] [pbrt output filename]\n");
    return 1;
  }

  std::vector<shape_t> shapes;
  const char *mtl_basepath = ""; // ?
  std::string errs = LoadObj(shapes, argv[1], mtl_basepath);

  if (errs.size() > 0) {
    fprintf(stderr, "Errors loading OBJ file: %s\n", errs.c_str());
    return 1;
  }

  FILE *f = (strcmp(argv[2], "-") == 0) ? stdout : fopen(argv[2], "w");
  if (!f) {
    perror(argv[2]);
    return 1;
  }

  float bounds[2][3] = { { 1e30, 1e30, 1e30 },{ -1e30, -1e30, -1e30 } };
  for (size_t i = 0; i < shapes.size(); ++i) {
    const shape_t &shape = shapes[i];
    const mesh_t &mesh = shape.mesh;
    for (size_t i = 0; i < mesh.positions.size(); ++i) {
      int c = i % 3;
      bounds[0][c] = std::min(bounds[0][c], mesh.positions[i]);
      bounds[1][c] = std::max(bounds[1][c], mesh.positions[i]);
    }
  }
  fprintf(f, "# Converted from \"%s\" by obj2pbrt\n", argv[1]);
  fprintf(f, "# Scene bounds: (%f, %f, %f) - (%f, %f, %f)\n\n\n",
         bounds[0][0], bounds[0][1], bounds[0][2],
         bounds[1][0], bounds[1][1], bounds[1][2]);

  int numAreaLights = 0;
  int numTriangles = 0;
  int numMeshes = shapes.size();

  for (size_t i = 0; i < shapes.size(); ++i) {
    const shape_t &shape = shapes[i];
    const material_t &mtl = shape.material;

    std::map<std::string, std::string>::const_iterator iter;
    for (iter = mtl.unknown_parameter.begin();
         iter != mtl.unknown_parameter.end(); ++iter)
      fprintf(stderr, "Unknown parameter: %s = %s\n", iter->first.c_str(),
              iter->second.c_str());

    fprintf(f, "# Name \"%s\"\n", shape.name.c_str());
    fprintf(f, "AttributeBegin\n");
    if (mtl.emission[0] > 0 || mtl.emission[1] > 0 || mtl.emission[2] > 0) {
      fprintf(f, "AreaLightSource \"area\" \"rgb L\" [ %f %f %f ]\n",
             mtl.emission[0], mtl.emission[1], mtl.emission[2]);
      ++numAreaLights;
    }

    fprintf(f,"# Material name \"%s\"\n", mtl.name.c_str());
    bool hasDiffuseTex = (mtl.diffuse_texname != "");
    if (mtl.diffuse_texname != "") {
      if (mtl.diffuse[0] != 0 || mtl.diffuse[1] != 0 || mtl.diffuse[2] != 0) {
          fprintf(f, "Texture \"%s-kd-img\" \"color\" \"imagemap\" "
                  "\"string filename\" [\"%s\"]\n",
                  mtl.name.c_str(), mtl.diffuse_texname.c_str());
          fprintf(f, "Texture \"%s-kd\" \"color\" \"scale\" \"texture tex1\" "
                  "\"%s-kd-img\" \"color tex2\" [%f %f %f]\n",
                  mtl.name.c_str(), mtl.name.c_str(),
                  mtl.diffuse[0], mtl.diffuse[1], mtl.diffuse[2]);
      }
      else {
          fprintf(f, "Texture \"%s-kd\" \"color\" \"imagemap\" "
                  "\"string filename\" [\"%s\"]\n",
                  mtl.name.c_str(), mtl.diffuse_texname.c_str());
      }
    }

    bool hasSpecularTex = (mtl.specular_texname != "");
    if (mtl.specular_texname != "") {
      if (mtl.specular[0] != 0 || mtl.specular[1] != 0 || mtl.specular[2] != 0) {
        fprintf(f, "Texture \"%s-ks-img\" \"color\" \"imagemap\" "
                "\"string filename\" [\"%s\"]\n",
                mtl.name.c_str(), mtl.specular_texname.c_str());
        fprintf(f, "Texture \"%s-ks\" \"color\" \"scale\" \"texture tex1\" "
                "\"%s-ks-img\" \"color tex2\" [%f %f %f]\n",
                mtl.name.c_str(), mtl.name.c_str(),
                mtl.specular[0], mtl.specular[1], mtl.specular[2]);
      }
      else {
        fprintf(f, "Texture \"%s-ks\" \"color\" \"imagemap\" "
                "\"string filename\" [\"%s\"]\n",
                mtl.name.c_str(), mtl.specular_texname.c_str());
      }
    }

    if (mtl.normal_texname != "") {
      fprintf(f, "Texture \"%s-bump-map\" \"float\" \"imagemap\" "
              "\"string filename\" [\"%s\"]\n",
              mtl.name.c_str(), mtl.normal_texname.c_str());
      fprintf(f, "Texture \"%s-bump\" \"float\" \"scale\" \"float tex1\" [%f] "
              "\"texture tex2\" \"%s-bump-map\"\n",
              mtl.name.c_str(), mtl.bump_scale, mtl.name.c_str());
    }

    float roughness = (mtl.shininess == 0) ? 0. : (1.f / mtl.shininess);
    fprintf(f, "Material \"uber\" ");
    if (hasDiffuseTex)
        fprintf(f, "\"texture Kd\" \"%s-kd\" ", mtl.name.c_str());
    else
        fprintf(f, "\"color Kd\" [%f %f %f] ", mtl.diffuse[0],
                mtl.diffuse[1], mtl.diffuse[2]);
    if (hasSpecularTex)
        fprintf(f, "\"texture Ks\" \"%s-ks\" ", mtl.name.c_str());
    else
        fprintf(f, "\"color Ks\" [%f %f %f] ", mtl.specular[0],
                mtl.specular[1], mtl.specular[2]);
    fprintf(f, "\"float roughness\" [%f] "
            "\"rgb Kt\" [%f %f %f] \"float index\" [%f] "
            "\"rgb opacity\" [%f %f %f] ",
            roughness, mtl.transmittance[0], mtl.transmittance[1],
            mtl.transmittance[2], mtl.ior, mtl.dissolve, mtl.dissolve,
            mtl.dissolve);
    if (mtl.normal_texname != "")
        fprintf(f, "\"texture bumpmap\" \"%s-bump\" ", mtl.name.c_str());
    fprintf(f, "\n\n");

    const mesh_t &mesh = shape.mesh;
    fprintf(f, "Shape \"trianglemesh\"\n");
    fprintf(f, "  \"point P\" [\n    ");
    for (size_t i = 0; i < mesh.positions.size(); ++i) {
      fprintf(f, "%.10g ", mesh.positions[i]);
      if (((i + 1) % 3) == 0)
        fprintf(f, "\n    ");
    }
    fprintf(f, "]\n");
    if (mesh.normals.size()) {
      fprintf(f, "  \"normal N\" [\n    ");
      for (size_t i = 0; i < mesh.normals.size(); ++i) {
        fprintf(f, "%.10g ", mesh.normals[i]);
        if (((i + 1) % 3) == 0)
          fprintf(f, "\n    ");
      }
      fprintf(f, "]\n");
    }
    if (mesh.texcoords.size()) {
      fprintf(f, "    \"float st\" [\n    ");
      for (size_t i = 0; i < mesh.texcoords.size(); ++i) {
        fprintf(f, "%.10g ", mesh.texcoords[i]);
        if (((i + 1) % 2) == 0)
          fprintf(f, "\n    ");
      }
      fprintf(f, "]\n");
    }
    numTriangles += mesh.indices.size() / 3;
    fprintf(f, "  \"integer indices\" [\n    ");
    for (size_t i = 0; i < mesh.indices.size(); ++i) {
      fprintf(f, "%d ", mesh.indices[i]);
      if (((i + 1) % 3) == 0)
        fprintf(f, "\n    ");
    }
    fprintf(f, "]\n");
    fprintf(f, "AttributeEnd\n\n\n");
  }
  if (f != stdout)
    fclose(f);

  fprintf(stderr, "Converted %d meshes (%d triangles, %d mesh emitters).\n",
          numMeshes, numTriangles, numAreaLights);

  return 0;
}

