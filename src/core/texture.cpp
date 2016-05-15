
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


// core/texture.cpp*
#include "texture.h"
#include "shape.h"

// Texture Inline Functions
inline Float SmoothStep(Float min, Float max, Float value) {
    Float v = Clamp((value - min) / (max - min), 0, 1);
    return v * v * (-2 * v + 3);
}

// Texture Forward Declarations
inline Float Grad(int x, int y, int z, Float dx, Float dy, Float dz);
inline Float NoiseWeight(Float t);

// Perlin Noise Data
static PBRT_CONSTEXPR int NoisePermSize = 256;
static int NoisePerm[2 * NoisePermSize] = {
    151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194, 233, 7, 225, 140,
    36, 103, 30, 69, 142,
    // Remainder of the noise permutation table
    8, 99, 37, 240, 21, 10, 23, 190, 6, 148, 247, 120, 234, 75, 0, 26, 197, 62,
    94, 252, 219, 203, 117, 35, 11, 32, 57, 177, 33, 88, 237, 149, 56, 87, 174,
    20, 125, 136, 171, 168, 68, 175, 74, 165, 71, 134, 139, 48, 27, 166, 77,
    146, 158, 231, 83, 111, 229, 122, 60, 211, 133, 230, 220, 105, 92, 41, 55,
    46, 245, 40, 244, 102, 143, 54, 65, 25, 63, 161, 1, 216, 80, 73, 209, 76,
    132, 187, 208, 89, 18, 169, 200, 196, 135, 130, 116, 188, 159, 86, 164, 100,
    109, 198, 173, 186, 3, 64, 52, 217, 226, 250, 124, 123, 5, 202, 38, 147,
    118, 126, 255, 82, 85, 212, 207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28,
    42, 223, 183, 170, 213, 119, 248, 152, 2, 44, 154, 163, 70, 221, 153, 101,
    155, 167, 43, 172, 9, 129, 22, 39, 253, 19, 98, 108, 110, 79, 113, 224, 232,
    178, 185, 112, 104, 218, 246, 97, 228, 251, 34, 242, 193, 238, 210, 144, 12,
    191, 179, 162, 241, 81, 51, 145, 235, 249, 14, 239, 107, 49, 192, 214, 31,
    181, 199, 106, 157, 184, 84, 204, 176, 115, 121, 50, 45, 127, 4, 150, 254,
    138, 236, 205, 93, 222, 114, 67, 29, 24, 72, 243, 141, 128, 195, 78, 66,
    215, 61, 156, 180, 151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194,
    233, 7, 225, 140, 36, 103, 30, 69, 142, 8, 99, 37, 240, 21, 10, 23, 190, 6,
    148, 247, 120, 234, 75, 0, 26, 197, 62, 94, 252, 219, 203, 117, 35, 11, 32,
    57, 177, 33, 88, 237, 149, 56, 87, 174, 20, 125, 136, 171, 168, 68, 175, 74,
    165, 71, 134, 139, 48, 27, 166, 77, 146, 158, 231, 83, 111, 229, 122, 60,
    211, 133, 230, 220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54, 65, 25,
    63, 161, 1, 216, 80, 73, 209, 76, 132, 187, 208, 89, 18, 169, 200, 196, 135,
    130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186, 3, 64, 52, 217, 226,
    250, 124, 123, 5, 202, 38, 147, 118, 126, 255, 82, 85, 212, 207, 206, 59,
    227, 47, 16, 58, 17, 182, 189, 28, 42, 223, 183, 170, 213, 119, 248, 152, 2,
    44, 154, 163, 70, 221, 153, 101, 155, 167, 43, 172, 9, 129, 22, 39, 253, 19,
    98, 108, 110, 79, 113, 224, 232, 178, 185, 112, 104, 218, 246, 97, 228, 251,
    34, 242, 193, 238, 210, 144, 12, 191, 179, 162, 241, 81, 51, 145, 235, 249,
    14, 239, 107, 49, 192, 214, 31, 181, 199, 106, 157, 184, 84, 204, 176, 115,
    121, 50, 45, 127, 4, 150, 254, 138, 236, 205, 93, 222, 114, 67, 29, 24, 72,
    243, 141, 128, 195, 78, 66, 215, 61, 156, 180};

// Texture Method Definitions
TextureMapping2D::~TextureMapping2D() { }
TextureMapping3D::~TextureMapping3D() { }

UVMapping2D::UVMapping2D(Float su, Float sv, Float du, Float dv)
    : su(su), sv(sv), du(du), dv(dv) {}
Point2f UVMapping2D::Map(const SurfaceInteraction &si, Vector2f *dstdx,
                         Vector2f *dstdy) const {
    // Compute texture differentials for 2D identity mapping
    *dstdx = Vector2f(su * si.dudx, sv * si.dvdx);
    *dstdy = Vector2f(su * si.dudy, sv * si.dvdy);
    return Point2f(su * si.uv[0] + du, sv * si.uv[1] + dv);
}

Point2f SphericalMapping2D::Map(const SurfaceInteraction &si, Vector2f *dstdx,
                                Vector2f *dstdy) const {
    Point2f st = sphere(si.p);
    // Compute texture coordinate differentials for sphere $(u,v)$ mapping
    const Float delta = .1f;
    Point2f stDeltaX = sphere(si.p + delta * si.dpdx);
    *dstdx = (stDeltaX - st) / delta;
    Point2f stDeltaY = sphere(si.p + delta * si.dpdy);
    *dstdy = (stDeltaY - st) / delta;

    // Handle sphere mapping discontinuity for coordinate differentials
    if ((*dstdx)[1] > .5)
        (*dstdx)[1] = 1 - (*dstdx)[1];
    else if ((*dstdx)[1] < -.5f)
        (*dstdx)[1] = -((*dstdx)[1] + 1);
    if ((*dstdy)[1] > .5)
        (*dstdy)[1] = 1 - (*dstdy)[1];
    else if ((*dstdy)[1] < -.5f)
        (*dstdy)[1] = -((*dstdy)[1] + 1);
    return st;
}

Point2f SphericalMapping2D::sphere(const Point3f &p) const {
    Vector3f vec = Normalize(WorldToTexture(p) - Point3f(0, 0, 0));
    Float theta = SphericalTheta(vec), phi = SphericalPhi(vec);
    return Point2f(theta * InvPi, phi * Inv2Pi);
}

Point2f CylindricalMapping2D::Map(const SurfaceInteraction &si, Vector2f *dstdx,
                                  Vector2f *dstdy) const {
    Point2f st = cylinder(si.p);
    // Compute texture coordinate differentials for cylinder $(u,v)$ mapping
    const Float delta = .01f;
    Point2f stDeltaX = cylinder(si.p + delta * si.dpdx);
    *dstdx = (stDeltaX - st) / delta;
    if ((*dstdx)[1] > .5)
        (*dstdx)[1] = 1.f - (*dstdx)[1];
    else if ((*dstdx)[1] < -.5f)
        (*dstdx)[1] = -((*dstdx)[1] + 1);
    Point2f stDeltaY = cylinder(si.p + delta * si.dpdy);
    *dstdy = (stDeltaY - st) / delta;
    if ((*dstdy)[1] > .5)
        (*dstdy)[1] = 1.f - (*dstdy)[1];
    else if ((*dstdy)[1] < -.5f)
        (*dstdy)[1] = -((*dstdy)[1] + 1);
    return st;
}

Point2f PlanarMapping2D::Map(const SurfaceInteraction &si, Vector2f *dstdx,
                             Vector2f *dstdy) const {
    Vector3f vec(si.p);
    *dstdx = Vector2f(Dot(si.dpdx, vs), Dot(si.dpdx, vt));
    *dstdy = Vector2f(Dot(si.dpdy, vs), Dot(si.dpdy, vt));
    return Point2f(ds + Dot(vec, vs), dt + Dot(vec, vt));
}

Point3f IdentityMapping3D::Map(const SurfaceInteraction &si, Vector3f *dpdx,
                               Vector3f *dpdy) const {
    *dpdx = WorldToTexture(si.dpdx);
    *dpdy = WorldToTexture(si.dpdy);
    return WorldToTexture(si.p);
}

Float Noise(Float x, Float y, Float z) {
    // Compute noise cell coordinates and offsets
    int ix = std::floor(x), iy = std::floor(y), iz = std::floor(z);
    Float dx = x - ix, dy = y - iy, dz = z - iz;

    // Compute gradient weights
    ix &= NoisePermSize - 1;
    iy &= NoisePermSize - 1;
    iz &= NoisePermSize - 1;
    Float w000 = Grad(ix, iy, iz, dx, dy, dz);
    Float w100 = Grad(ix + 1, iy, iz, dx - 1, dy, dz);
    Float w010 = Grad(ix, iy + 1, iz, dx, dy - 1, dz);
    Float w110 = Grad(ix + 1, iy + 1, iz, dx - 1, dy - 1, dz);
    Float w001 = Grad(ix, iy, iz + 1, dx, dy, dz - 1);
    Float w101 = Grad(ix + 1, iy, iz + 1, dx - 1, dy, dz - 1);
    Float w011 = Grad(ix, iy + 1, iz + 1, dx, dy - 1, dz - 1);
    Float w111 = Grad(ix + 1, iy + 1, iz + 1, dx - 1, dy - 1, dz - 1);

    // Compute trilinear interpolation of weights
    Float wx = NoiseWeight(dx), wy = NoiseWeight(dy), wz = NoiseWeight(dz);
    Float x00 = Lerp(wx, w000, w100);
    Float x10 = Lerp(wx, w010, w110);
    Float x01 = Lerp(wx, w001, w101);
    Float x11 = Lerp(wx, w011, w111);
    Float y0 = Lerp(wy, x00, x10);
    Float y1 = Lerp(wy, x01, x11);
    return Lerp(wz, y0, y1);
}

Float Noise(const Point3f &p) { return Noise(p.x, p.y, p.z); }
inline Float Grad(int x, int y, int z, Float dx, Float dy, Float dz) {
    int h = NoisePerm[NoisePerm[NoisePerm[x] + y] + z];
    h &= 15;
    Float u = h < 8 || h == 12 || h == 13 ? dx : dy;
    Float v = h < 4 || h == 12 || h == 13 ? dy : dz;
    return ((h & 1) ? -u : u) + ((h & 2) ? -v : v);
}

inline Float NoiseWeight(Float t) {
    Float t3 = t * t * t;
    Float t4 = t3 * t;
    return 6 * t4 * t - 15 * t4 + 10 * t3;
}

Float FBm(const Point3f &p, const Vector3f &dpdx, const Vector3f &dpdy,
          Float omega, int maxOctaves) {
    // Compute number of octaves for antialiased FBm
    Float len2 = std::max(dpdx.LengthSquared(), dpdy.LengthSquared());
    Float n = Clamp(-1 - .5f * Log2(len2), 0, maxOctaves);
    int nInt = std::floor(n);

    // Compute sum of octaves of noise for FBm
    Float sum = 0, lambda = 1, o = 1;
    for (int i = 0; i < nInt; ++i) {
        sum += o * Noise(lambda * p);
        lambda *= 1.99f;
        o *= omega;
    }
    Float nPartial = n - nInt;
    sum += o * SmoothStep(.3f, .7f, nPartial) * Noise(lambda * p);
    return sum;
}

Float Turbulence(const Point3f &p, const Vector3f &dpdx, const Vector3f &dpdy,
                 Float omega, int maxOctaves) {
    // Compute number of octaves for antialiased FBm
    Float len2 = std::max(dpdx.LengthSquared(), dpdy.LengthSquared());
    Float n = Clamp(-1 - .5f * Log2(len2), 0, maxOctaves);
    int nInt = std::floor(n);

    // Compute sum of octaves of noise for turbulence
    Float sum = 0, lambda = 1, o = 1;
    for (int i = 0; i < nInt; ++i) {
        sum += o * std::abs(Noise(lambda * p));
        lambda *= 1.99f;
        o *= omega;
    }

    // Account for contributions of clamped octaves in turbulence
    Float nPartial = n - nInt;
    sum += o * Lerp(SmoothStep(.3f, .7f, nPartial), 0.2,
                    std::abs(Noise(lambda * p)));
    for (int i = nInt; i < maxOctaves; ++i) {
        sum += o * 0.2f;
        o *= omega;
    }
    return sum;
}

// Texture Function Definitions
Float Lanczos(Float x, Float tau) {
    x = std::abs(x);
    if (x < 1e-5f) return 1;
    if (x > 1.f) return 0;
    x *= Pi;
    Float s = std::sin(x * tau) / (x * tau);
    Float lanczos = std::sin(x) / x;
    return s * lanczos;
}
