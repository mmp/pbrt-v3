
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

#include "stdafx.h"

// core/api.cpp*
#include "api.h"
#include "parallel.h"
#include "paramset.h"
#include "spectrum.h"
#include "scene.h"
#include "film.h"
#include "medium.h"
#include "stats.h"

// API Additional Headers
#include "accelerators/bvh.h"
#include "accelerators/kdtreeaccel.h"
#include "cameras/environment.h"
#include "cameras/orthographic.h"
#include "cameras/perspective.h"
#include "cameras/realistic.h"
#include "filters/box.h"
#include "filters/gaussian.h"
#include "filters/mitchell.h"
#include "filters/sinc.h"
#include "filters/triangle.h"
#include "integrators/bdpt.h"
#include "integrators/directlighting.h"
#include "integrators/mlt.h"
#include "integrators/path.h"
#include "integrators/sppm.h"
#include "integrators/volpath.h"
#include "integrators/whitted.h"
#include "lights/diffuse.h"
#include "lights/distant.h"
#include "lights/goniometric.h"
#include "lights/infinite.h"
#include "lights/point.h"
#include "lights/projection.h"
#include "lights/spot.h"
#include "materials/fourier.h"
#include "materials/glass.h"
#include "materials/hair.h"
#include "materials/kdsubsurface.h"
#include "materials/matte.h"
#include "materials/metal.h"
#include "materials/mirror.h"
#include "materials/mixmat.h"
#include "materials/plastic.h"
#include "materials/substrate.h"
#include "materials/subsurface.h"
#include "materials/translucent.h"
#include "materials/uber.h"
#include "samplers/halton.h"
#include "samplers/maxmin.h"
#include "samplers/random.h"
#include "samplers/sobol.h"
#include "samplers/stratified.h"
#include "samplers/zerotwosequence.h"
#include "shapes/cone.h"
#include "shapes/curve.h"
#include "shapes/cylinder.h"
#include "shapes/disk.h"
#include "shapes/heightfield.h"
#include "shapes/hyperboloid.h"
#include "shapes/loopsubdiv.h"
#include "shapes/nurbs.h"
#include "shapes/paraboloid.h"
#include "shapes/sphere.h"
#include "shapes/triangle.h"
#include "shapes/plymesh.h"
#include "textures/bilerp.h"
#include "textures/checkerboard.h"
#include "textures/constant.h"
#include "textures/dots.h"
#include "textures/fbm.h"
#include "textures/imagemap.h"
#include "textures/marble.h"
#include "textures/mix.h"
#include "textures/scale.h"
#include "textures/uv.h"
#include "textures/windy.h"
#include "textures/wrinkled.h"
#include "media/grid.h"
#include "media/homogeneous.h"
#include <map>
#if (_MSC_VER >= 1400)
#include <stdio.h>
#define snprintf _snprintf
#endif

// API Global Variables
Options PbrtOptions;

// API Local Classes
constexpr int MaxTransforms = 2;
constexpr int StartTransformBits = 1 << 0;
constexpr int EndTransformBits = 1 << 1;
constexpr int AllTransformsBits = (1 << MaxTransforms) - 1;
struct TransformSet {
    // TransformSet Public Methods
    Transform &operator[](int i) {
        Assert(i >= 0 && i < MaxTransforms);
        return t[i];
    }
    const Transform &operator[](int i) const {
        Assert(i >= 0 && i < MaxTransforms);
        return t[i];
    }
    friend TransformSet Inverse(const TransformSet &ts) {
        TransformSet tInv;
        for (int i = 0; i < MaxTransforms; ++i) tInv.t[i] = Inverse(ts.t[i]);
        return tInv;
    }
    bool IsAnimated() const {
        for (int i = 0; i < MaxTransforms - 1; ++i)
            if (t[i] != t[i + 1]) return true;
        return false;
    }

  private:
    Transform t[MaxTransforms];
};

struct RenderOptions {
    // RenderOptions Public Methods
    RenderOptions();
    Scene *MakeScene();
    Camera *MakeCamera() const;
    Integrator *MakeIntegrator() const;

    // RenderOptions Public Data
    Float transformStartTime, transformEndTime;
    std::string FilterName;
    ParamSet FilterParams;
    std::string FilmName;
    ParamSet FilmParams;
    std::string SamplerName;
    ParamSet SamplerParams;
    std::string AcceleratorName;
    ParamSet AcceleratorParams;
    std::string IntegratorName;
    ParamSet IntegratorParams;
    std::string CameraName;
    ParamSet CameraParams;
    TransformSet CameraToWorld;
    std::map<std::string, std::shared_ptr<Medium>> namedMedia;
    std::vector<std::shared_ptr<Light>> lights;
    std::vector<std::shared_ptr<Primitive>> primitives;
    std::map<std::string, std::vector<std::shared_ptr<Primitive>>> instances;
    std::vector<std::shared_ptr<Primitive>> *currentInstance;
};

RenderOptions::RenderOptions() {
    // RenderOptions Constructor Implementation
    transformStartTime = 0.f;
    transformEndTime = 1.f;
    FilterName = "box";
    FilmName = "image";
    SamplerName = "lowdiscrepancy";
    AcceleratorName = "bvh";
    IntegratorName = "directlighting";
    CameraName = "perspective";
    currentInstance = nullptr;
}

struct GraphicsState {
    // Graphics State Methods
    GraphicsState();
    std::shared_ptr<Material> CreateMaterial(const ParamSet &params);
    MediumInterface CreateMediumInterface();

    // Graphics State
    std::string currentInsideMedium;
    std::string currentOutsideMedium;
    std::map<std::string, std::shared_ptr<Texture<Float>>> floatTextures;
    std::map<std::string, std::shared_ptr<Texture<Spectrum>>> spectrumTextures;
    ParamSet materialParams;
    std::string material;
    std::map<std::string, std::shared_ptr<Material>> namedMaterials;
    std::string currentNamedMaterial;
    ParamSet areaLightParams;
    std::string areaLight;
    bool reverseOrientation;
};

GraphicsState::GraphicsState() {
    // GraphicsState Constructor Implementation
    material = "matte";
    reverseOrientation = false;
}

class TransformCache {
  public:
    // TransformCache Public Methods
    void Lookup(const Transform &t, Transform **tCached,
                Transform **tCachedInverse) {
        auto iter = cache.find(t);
        if (iter == cache.end()) {
            Transform *tr = arena.Alloc<Transform>();
            *tr = t;
            Transform *tinv = arena.Alloc<Transform>();
            *tinv = Transform(Inverse(t));
            cache[t] = std::make_pair(tr, tinv);
            iter = cache.find(t);
        }
        if (tCached) *tCached = iter->second.first;
        if (tCachedInverse) *tCachedInverse = iter->second.second;
    }
    void Clear() {
        arena.Reset();
        cache.erase(cache.begin(), cache.end());
    }

  private:
    // TransformCache Private Data
    std::map<Transform, std::pair<Transform *, Transform *>> cache;
    MemoryArena arena;
};

// API Static Data
enum class APIState { Uninitialized, OptionsBlock, WorldBlock };
static APIState currentApiState = APIState::Uninitialized;
static TransformSet curTransform;
static int activeTransformBits = AllTransformsBits;
static std::map<std::string, TransformSet> namedCoordinateSystems;
static std::unique_ptr<RenderOptions> renderOptions;
static GraphicsState graphicsState;
static std::vector<GraphicsState> pushedGraphicsStates;
static std::vector<TransformSet> pushedTransforms;
static std::vector<uint32_t> pushedActiveTransformBits;
static TransformCache transformCache;

// API Macros
#define VERIFY_INITIALIZED(func)                           \
    if (currentApiState == APIState::Uninitialized) {      \
        Error(                                             \
            "pbrtInit() must be before calling \"%s()\". " \
            "Ignoring.",                                   \
            func);                                         \
        return;                                            \
    } else /* swallow trailing semicolon */
#define VERIFY_OPTIONS(func)                             \
    VERIFY_INITIALIZED(func);                            \
    if (currentApiState == APIState::WorldBlock) {       \
        Error(                                           \
            "Options cannot be set inside world block; " \
            "\"%s\" not allowed.  Ignoring.",            \
            func);                                       \
        return;                                          \
    } else /* swallow trailing semicolon */
#define VERIFY_WORLD(func)                                   \
    VERIFY_INITIALIZED(func);                                \
    if (currentApiState == APIState::OptionsBlock) {         \
        Error(                                               \
            "Scene description must be inside world block; " \
            "\"%s\" not allowed. Ignoring.",                 \
            func);                                           \
        return;                                              \
    } else /* swallow trailing semicolon */
#define FOR_ACTIVE_TRANSFORMS(expr)           \
    for (int i = 0; i < MaxTransforms; ++i)   \
        if (activeTransformBits & (1 << i)) { \
            expr                              \
        }
#define WARN_IF_ANIMATED_TRANSFORM(func)                             \
    do {                                                             \
        if (curTransform.IsAnimated())                               \
            Warning(                                                 \
                "Animated transformations set; ignoring for \"%s\" " \
                "and using the start transform only",                \
                func);                                               \
    } while (false)

// Object Creation Function Definitions
std::vector<std::shared_ptr<Shape>> MakeShapes(const std::string &name,
                                               const Transform *object2world,
                                               const Transform *world2object,
                                               bool reverseOrientation,
                                               const ParamSet &paramSet) {
    std::vector<std::shared_ptr<Shape>> shapes;
    std::shared_ptr<Shape> s;
    if (name == "sphere")
        s = CreateSphereShape(object2world, world2object, reverseOrientation,
                              paramSet);
    // Create remaining single _Shape_ types
    else if (name == "cylinder")
        s = CreateCylinderShape(object2world, world2object, reverseOrientation,
                                paramSet);
    else if (name == "disk")
        s = CreateDiskShape(object2world, world2object, reverseOrientation,
                            paramSet);
    else if (name == "cone")
        s = CreateConeShape(object2world, world2object, reverseOrientation,
                            paramSet);
    else if (name == "paraboloid")
        s = CreateParaboloidShape(object2world, world2object,
                                  reverseOrientation, paramSet);
    else if (name == "hyperboloid")
        s = CreateHyperboloidShape(object2world, world2object,
                                   reverseOrientation, paramSet);
    if (s != nullptr) shapes.push_back(s);

    // Create multiple-_Shape_ types
    else if (name == "curve")
        shapes = CreateCurveShape(object2world, world2object,
                                  reverseOrientation, paramSet);
    else if (name == "trianglemesh")
        shapes = CreateTriangleMeshShape(object2world, world2object,
                                         reverseOrientation, paramSet,
                                         &graphicsState.floatTextures);
    else if (name == "plymesh")
        shapes = CreatePLYMesh(object2world, world2object, reverseOrientation,
                               paramSet, &graphicsState.floatTextures);
    else if (name == "heightfield")
        shapes = CreateHeightfield(object2world, world2object,
                                   reverseOrientation, paramSet);
    else if (name == "loopsubdiv")
        shapes = CreateLoopSubdiv(object2world, world2object,
                                  reverseOrientation, paramSet);
    else if (name == "nurbs")
        shapes = CreateNURBS(object2world, world2object, reverseOrientation,
                             paramSet);
    else
        Warning("Shape \"%s\" unknown.", name.c_str());
    paramSet.ReportUnused();
    return shapes;
}

std::shared_ptr<Material> MakeMaterial(const std::string &name,
                                       const TextureParams &mp) {
    Material *material = nullptr;
    if (name == "" || name == "none")
        return nullptr;
    else if (name == "matte")
        material = CreateMatteMaterial(mp);
    else if (name == "plastic")
        material = CreatePlasticMaterial(mp);
    else if (name == "translucent")
        material = CreateTranslucentMaterial(mp);
    else if (name == "glass")
        material = CreateGlassMaterial(mp);
    else if (name == "hair")
        material = CreateHairMaterial(mp);
    else if (name == "mirror")
        material = CreateMirrorMaterial(mp);
    else if (name == "mix") {
        std::string m1 = mp.FindString("namedmaterial1", "");
        std::string m2 = mp.FindString("namedmaterial2", "");
        std::shared_ptr<Material> mat1 = graphicsState.namedMaterials[m1];
        std::shared_ptr<Material> mat2 = graphicsState.namedMaterials[m2];
        if (!mat1) {
            Error("Named material \"%s\" undefined.  Using \"matte\"",
                  m1.c_str());
            mat1 = MakeMaterial("matte", mp);
        }
        if (!mat2) {
            Error("Named material \"%s\" undefined.  Using \"matte\"",
                  m2.c_str());
            mat2 = MakeMaterial("matte", mp);
        }

        material = CreateMixMaterial(mp, mat1, mat2);
    } else if (name == "metal")
        material = CreateMetalMaterial(mp);
    else if (name == "substrate")
        material = CreateSubstrateMaterial(mp);
    else if (name == "uber")
        material = CreateUberMaterial(mp);
    else if (name == "subsurface")
        material = CreateSubsurfaceMaterial(mp);
    else if (name == "kdsubsurface")
        material = CreateKdSubsurfaceMaterial(mp);
    else if (name == "fourier")
        material = CreateFourierMaterial(mp);
    else
        Warning("Material \"%s\" unknown.", name.c_str());
    mp.ReportUnused();
    if (!material) Error("Unable to create material \"%s\"", name.c_str());
    return std::shared_ptr<Material>(material);
}

std::shared_ptr<Texture<Float>> MakeFloatTexture(const std::string &name,
                                                 const Transform &tex2world,
                                                 const TextureParams &tp) {
    Texture<Float> *tex = nullptr;
    if (name == "constant")
        tex = CreateConstantFloatTexture(tex2world, tp);
    else if (name == "scale")
        tex = CreateScaleFloatTexture(tex2world, tp);
    else if (name == "mix")
        tex = CreateMixFloatTexture(tex2world, tp);
    else if (name == "bilerp")
        tex = CreateBilerpFloatTexture(tex2world, tp);
    else if (name == "imagemap")
        tex = CreateImageFloatTexture(tex2world, tp);
    else if (name == "uv")
        tex = CreateUVFloatTexture(tex2world, tp);
    else if (name == "checkerboard")
        tex = CreateCheckerboardFloatTexture(tex2world, tp);
    else if (name == "dots")
        tex = CreateDotsFloatTexture(tex2world, tp);
    else if (name == "fbm")
        tex = CreateFBmFloatTexture(tex2world, tp);
    else if (name == "wrinkled")
        tex = CreateWrinkledFloatTexture(tex2world, tp);
    else if (name == "marble")
        tex = CreateMarbleFloatTexture(tex2world, tp);
    else if (name == "windy")
        tex = CreateWindyFloatTexture(tex2world, tp);
    else
        Warning("Float texture \"%s\" unknown.", name.c_str());
    tp.ReportUnused();
    return std::shared_ptr<Texture<Float>>(tex);
}

std::shared_ptr<Texture<Spectrum>> MakeSpectrumTexture(
    const std::string &name, const Transform &tex2world,
    const TextureParams &tp) {
    Texture<Spectrum> *tex = nullptr;
    if (name == "constant")
        tex = CreateConstantSpectrumTexture(tex2world, tp);
    else if (name == "scale")
        tex = CreateScaleSpectrumTexture(tex2world, tp);
    else if (name == "mix")
        tex = CreateMixSpectrumTexture(tex2world, tp);
    else if (name == "bilerp")
        tex = CreateBilerpSpectrumTexture(tex2world, tp);
    else if (name == "imagemap")
        tex = CreateImageSpectrumTexture(tex2world, tp);
    else if (name == "uv")
        tex = CreateUVSpectrumTexture(tex2world, tp);
    else if (name == "checkerboard")
        tex = CreateCheckerboardSpectrumTexture(tex2world, tp);
    else if (name == "dots")
        tex = CreateDotsSpectrumTexture(tex2world, tp);
    else if (name == "fbm")
        tex = CreateFBmSpectrumTexture(tex2world, tp);
    else if (name == "wrinkled")
        tex = CreateWrinkledSpectrumTexture(tex2world, tp);
    else if (name == "marble")
        tex = CreateMarbleSpectrumTexture(tex2world, tp);
    else if (name == "windy")
        tex = CreateWindySpectrumTexture(tex2world, tp);
    else
        Warning("Spectrum texture \"%s\" unknown.", name.c_str());
    tp.ReportUnused();
    return std::shared_ptr<Texture<Spectrum>>(tex);
}

std::shared_ptr<Medium> MakeMedium(const std::string &name,
                                   const ParamSet &paramSet,
                                   const Transform &medium2world) {
    Float sig_a_rgb[3] = {.0011f, .0024f, .014f},
          sig_s_rgb[3] = {2.55f, 3.21f, 3.77f};
    Spectrum sig_a = Spectrum::FromRGB(sig_a_rgb),
             sig_s = Spectrum::FromRGB(sig_s_rgb);
    std::string preset = paramSet.FindOneString("preset", "");
    bool found = GetMediumScatteringProperties(preset, &sig_a, &sig_s);
    if (preset != "" && !found)
        Warning("Material preset \"%s\" not found.  Using defaults.",
                preset.c_str());
    Float scale = paramSet.FindOneFloat("scale", 1.f);
    Float g = paramSet.FindOneFloat("g", 0.0f);
    sig_a = paramSet.FindOneSpectrum("sigma_a", sig_a) * scale;
    sig_s = paramSet.FindOneSpectrum("sigma_s", sig_s) * scale;
    Medium *m = NULL;
    if (name == "homogeneous") {
        m = new HomogeneousMedium(sig_a, sig_s, g);
    } else if (name == "heterogeneous") {
        int nitems;
        const Float *data = paramSet.FindFloat("density", &nitems);
        if (!data) {
            Error("No \"density\" values provided for heterogeneous medium?");
            return NULL;
        }
        int nx = paramSet.FindOneInt("nx", 1);
        int ny = paramSet.FindOneInt("ny", 1);
        int nz = paramSet.FindOneInt("nz", 1);
        Point3f p0 = paramSet.FindOnePoint3f("p0", Point3f(0.f, 0.f, 0.f));
        Point3f p1 = paramSet.FindOnePoint3f("p1", Point3f(1.f, 1.f, 1.f));
        if (nitems != nx * ny * nz) {
            Error(
                "GridDensityMedium has %d density values; expected nx*ny*nz = "
                "%d",
                nitems, nx * ny * nz);
            return NULL;
        }
        Transform data2Medium = Translate(Vector3f(p0)) *
                                Scale(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
        m = new GridDensityMedium(sig_a, sig_s, g, nx, ny, nz,
                                  medium2world * data2Medium, data);
    } else
        Warning("Medium \"%s\" unknown.", name.c_str());
    paramSet.ReportUnused();
    return std::shared_ptr<Medium>(m);
}

std::shared_ptr<Light> MakeLight(const std::string &name,
                                 const ParamSet &paramSet,
                                 const Transform &light2world,
                                 const MediumInterface &mediumInterface) {
    std::shared_ptr<Light> light;
    if (name == "point")
        light =
            CreatePointLight(light2world, mediumInterface.outside, paramSet);
    else if (name == "spot")
        light = CreateSpotLight(light2world, mediumInterface.outside, paramSet);
    else if (name == "goniometric")
        light = CreateGoniometricLight(light2world, mediumInterface.outside,
                                       paramSet);
    else if (name == "projection")
        light = CreateProjectionLight(light2world, mediumInterface.outside,
                                      paramSet);
    else if (name == "distant")
        light = CreateDistantLight(light2world, paramSet);
    else if (name == "infinite" || name == "exinfinite")
        light = CreateInfiniteLight(light2world, paramSet);
    else
        Warning("Light \"%s\" unknown.", name.c_str());
    paramSet.ReportUnused();
    return light;
}

std::shared_ptr<AreaLight> MakeAreaLight(const std::string &name,
                                         const Transform &light2world,
                                         const MediumInterface &mediumInterface,
                                         const ParamSet &paramSet,
                                         const std::shared_ptr<Shape> &shape) {
    std::shared_ptr<AreaLight> area;
    if (name == "area" || name == "diffuse")
        area = CreateDiffuseAreaLight(light2world, mediumInterface.outside,
                                      paramSet, shape);
    else
        Warning("Area light \"%s\" unknown.", name.c_str());
    paramSet.ReportUnused();
    return area;
}

Integrator *MakeIntegrator(const std::string &name, const ParamSet &paramSet) {
    Integrator *si = nullptr;

    paramSet.ReportUnused();
    return si;
}

std::shared_ptr<Primitive> MakeAccelerator(
    const std::string &name,
    const std::vector<std::shared_ptr<Primitive>> &prims,
    const ParamSet &paramSet) {
    std::shared_ptr<Primitive> accel;
    if (name == "bvh")
        accel = CreateBVHAccelerator(prims, paramSet);
    else if (name == "kdtree")
        accel = CreateKdTreeAccelerator(prims, paramSet);
    else
        Warning("Accelerator \"%s\" unknown.", name.c_str());
    paramSet.ReportUnused();
    return accel;
}

Camera *MakeCamera(const std::string &name, const ParamSet &paramSet,
                   const TransformSet &cam2worldSet, Float transformStart,
                   Float transformEnd, Film *film) {
    Camera *camera = nullptr;
    MediumInterface mediumInterface = graphicsState.CreateMediumInterface();
    Assert(MaxTransforms == 2);
    Transform *cam2world[2];
    transformCache.Lookup(cam2worldSet[0], &cam2world[0], nullptr);
    transformCache.Lookup(cam2worldSet[1], &cam2world[1], nullptr);
    AnimatedTransform animatedCam2World(cam2world[0], transformStart,
                                        cam2world[1], transformEnd);
    if (name == "perspective")
        camera = CreatePerspectiveCamera(paramSet, animatedCam2World, film,
                                         mediumInterface.outside);
    else if (name == "orthographic")
        camera = CreateOrthographicCamera(paramSet, animatedCam2World, film,
                                          mediumInterface.outside);
    else if (name == "realistic")
        camera = CreateRealisticCamera(paramSet, animatedCam2World, film,
                                       mediumInterface.outside);
    else if (name == "environment")
        camera = CreateEnvironmentCamera(paramSet, animatedCam2World, film,
                                         mediumInterface.outside);
    else
        Warning("Camera \"%s\" unknown.", name.c_str());
    paramSet.ReportUnused();
    return camera;
}

std::shared_ptr<Sampler> MakeSampler(const std::string &name,
                                     const ParamSet &paramSet,
                                     const Film *film) {
    Sampler *sampler = nullptr;
    if (name == "lowdiscrepancy" || name == "02sequence")
        sampler = CreateZeroTwoSequenceSampler(paramSet);
    else if (name == "maxmindist")
        sampler = CreateMaxMinDistSampler(paramSet);
    else if (name == "halton")
        sampler = CreateHaltonSampler(paramSet, film->GetSampleBounds());
    else if (name == "sobol")
        sampler = CreateSobolSampler(paramSet, film->GetSampleBounds());
    else if (name == "random")
        sampler = CreateRandomSampler(paramSet);
    else if (name == "stratified")
        sampler = CreateStratifiedSampler(paramSet);
    else
        Warning("Sampler \"%s\" unknown.", name.c_str());
    paramSet.ReportUnused();
    return std::shared_ptr<Sampler>(sampler);
}

Filter *MakeFilter(const std::string &name, const ParamSet &paramSet) {
    Filter *filter = nullptr;
    if (name == "box")
        filter = CreateBoxFilter(paramSet);
    else if (name == "gaussian")
        filter = CreateGaussianFilter(paramSet);
    else if (name == "mitchell")
        filter = CreateMitchellFilter(paramSet);
    else if (name == "sinc")
        filter = CreateSincFilter(paramSet);
    else if (name == "triangle")
        filter = CreateTriangleFilter(paramSet);
    else
        Warning("Filter \"%s\" unknown.", name.c_str());
    paramSet.ReportUnused();
    return filter;
}

Film *MakeFilm(const std::string &name, const ParamSet &paramSet,
               Filter *filter) {
    Film *film = nullptr;
    if (name == "image")
        film = CreateFilm(paramSet, filter);
    else
        Warning("Film \"%s\" unknown.", name.c_str());
    paramSet.ReportUnused();
    return film;
}

// API Function Definitions
void pbrtInit(const Options &opt) {
    PbrtOptions = opt;
    // API Initialization
    if (currentApiState != APIState::Uninitialized)
        Error("pbrtInit() has already been called.");
    currentApiState = APIState::OptionsBlock;
    renderOptions.reset(new RenderOptions);
    graphicsState = GraphicsState();

    // General \pbrt Initialization
    SampledSpectrum::Init();
    InitProfiler();
}

void pbrtCleanup() {
    // API Cleanup
    if (currentApiState == APIState::Uninitialized)
        Error("pbrtCleanup() called without pbrtInit().");
    else if (currentApiState == APIState::WorldBlock)
        Error("pbrtCleanup() called while inside world block.");
    currentApiState = APIState::Uninitialized;
    TerminateWorkerThreads();
    renderOptions.reset(nullptr);
}

void pbrtIdentity() {
    VERIFY_INITIALIZED("Identity");
    FOR_ACTIVE_TRANSFORMS(curTransform[i] = Transform();)
}

void pbrtTranslate(Float dx, Float dy, Float dz) {
    VERIFY_INITIALIZED("Translate");
    FOR_ACTIVE_TRANSFORMS(curTransform[i] = curTransform[i] *
                                            Translate(Vector3f(dx, dy, dz));)
}

void pbrtTransform(Float tr[16]) {
    VERIFY_INITIALIZED("Transform");
    FOR_ACTIVE_TRANSFORMS(
        curTransform[i] = Transform(Matrix4x4(
            tr[0], tr[4], tr[8], tr[12], tr[1], tr[5], tr[9], tr[13], tr[2],
            tr[6], tr[10], tr[14], tr[3], tr[7], tr[11], tr[15]));)
}

void pbrtConcatTransform(Float tr[16]) {
    VERIFY_INITIALIZED("ConcatTransform");
    FOR_ACTIVE_TRANSFORMS(
        curTransform[i] =
            curTransform[i] *
            Transform(Matrix4x4(tr[0], tr[4], tr[8], tr[12], tr[1], tr[5],
                                tr[9], tr[13], tr[2], tr[6], tr[10], tr[14],
                                tr[3], tr[7], tr[11], tr[15]));)
}

void pbrtRotate(Float angle, Float dx, Float dy, Float dz) {
    VERIFY_INITIALIZED("Rotate");
    FOR_ACTIVE_TRANSFORMS(curTransform[i] =
                              curTransform[i] *
                              Rotate(angle, Vector3f(dx, dy, dz));)
}

void pbrtScale(Float sx, Float sy, Float sz) {
    VERIFY_INITIALIZED("Scale");
    FOR_ACTIVE_TRANSFORMS(curTransform[i] =
                              curTransform[i] * Scale(sx, sy, sz);)
}

void pbrtLookAt(Float ex, Float ey, Float ez, Float lx, Float ly, Float lz,
                Float ux, Float uy, Float uz) {
    VERIFY_INITIALIZED("LookAt");
    Transform lookAt =
        LookAt(Point3f(ex, ey, ez), Point3f(lx, ly, lz), Vector3f(ux, uy, uz));
    FOR_ACTIVE_TRANSFORMS(curTransform[i] = curTransform[i] * lookAt;);
}

void pbrtCoordinateSystem(const std::string &name) {
    VERIFY_INITIALIZED("CoordinateSystem");
    namedCoordinateSystems[name] = curTransform;
}

void pbrtCoordSysTransform(const std::string &name) {
    VERIFY_INITIALIZED("CoordSysTransform");
    if (namedCoordinateSystems.find(name) != namedCoordinateSystems.end())
        curTransform = namedCoordinateSystems[name];
    else
        Warning("Couldn't find named coordinate system \"%s\"", name.c_str());
}

void pbrtActiveTransformAll() { activeTransformBits = AllTransformsBits; }

void pbrtActiveTransformEndTime() { activeTransformBits = EndTransformBits; }

void pbrtActiveTransformStartTime() {
    activeTransformBits = StartTransformBits;
}

void pbrtTransformTimes(Float start, Float end) {
    VERIFY_OPTIONS("TransformTimes");
    renderOptions->transformStartTime = start;
    renderOptions->transformEndTime = end;
}

void pbrtPixelFilter(const std::string &name, const ParamSet &params) {
    VERIFY_OPTIONS("PixelFilter");
    renderOptions->FilterName = name;
    renderOptions->FilterParams = params;
}

void pbrtFilm(const std::string &type, const ParamSet &params) {
    VERIFY_OPTIONS("Film");
    renderOptions->FilmParams = params;
    renderOptions->FilmName = type;
}

void pbrtSampler(const std::string &name, const ParamSet &params) {
    VERIFY_OPTIONS("Sampler");
    renderOptions->SamplerName = name;
    renderOptions->SamplerParams = params;
}

void pbrtAccelerator(const std::string &name, const ParamSet &params) {
    VERIFY_OPTIONS("Accelerator");
    renderOptions->AcceleratorName = name;
    renderOptions->AcceleratorParams = params;
}

void pbrtIntegrator(const std::string &name, const ParamSet &params) {
    VERIFY_OPTIONS("Integrator");
    renderOptions->IntegratorName = name;
    renderOptions->IntegratorParams = params;
}

void pbrtCamera(const std::string &name, const ParamSet &params) {
    VERIFY_OPTIONS("Camera");
    renderOptions->CameraName = name;
    renderOptions->CameraParams = params;
    renderOptions->CameraToWorld = Inverse(curTransform);
    namedCoordinateSystems["camera"] = renderOptions->CameraToWorld;
}

void pbrtMakeNamedMedium(const std::string &name, const ParamSet &params) {
    VERIFY_INITIALIZED("MakeNamedMedium");
    WARN_IF_ANIMATED_TRANSFORM("MakeNamedMedium");
    std::string type = params.FindOneString("type", "");
    if (type == "")
        Error("No parameter string \"type\" found in MakeNamedMedium");
    else {
        std::shared_ptr<Medium> medium =
            MakeMedium(type, params, curTransform[0]);
        if (medium) renderOptions->namedMedia[name] = medium;
    }
}

void pbrtMediumInterface(const std::string &insideName,
                         const std::string &outsideName) {
    VERIFY_INITIALIZED("MediumInterface");
    graphicsState.currentInsideMedium = insideName;
    graphicsState.currentOutsideMedium = outsideName;
}

void pbrtWorldBegin() {
    VERIFY_OPTIONS("WorldBegin");
    currentApiState = APIState::WorldBlock;
    for (int i = 0; i < MaxTransforms; ++i) curTransform[i] = Transform();
    activeTransformBits = AllTransformsBits;
    namedCoordinateSystems["world"] = curTransform;
}

void pbrtAttributeBegin() {
    VERIFY_WORLD("AttributeBegin");
    pushedGraphicsStates.push_back(graphicsState);
    pushedTransforms.push_back(curTransform);
    pushedActiveTransformBits.push_back(activeTransformBits);
}

void pbrtAttributeEnd() {
    VERIFY_WORLD("AttributeEnd");
    if (!pushedGraphicsStates.size()) {
        Error(
            "Unmatched pbrtAttributeEnd() encountered. "
            "Ignoring it.");
        return;
    }
    graphicsState = pushedGraphicsStates.back();
    pushedGraphicsStates.pop_back();
    curTransform = pushedTransforms.back();
    pushedTransforms.pop_back();
    activeTransformBits = pushedActiveTransformBits.back();
    pushedActiveTransformBits.pop_back();
}

void pbrtTransformBegin() {
    VERIFY_WORLD("TransformBegin");
    pushedTransforms.push_back(curTransform);
    pushedActiveTransformBits.push_back(activeTransformBits);
}

void pbrtTransformEnd() {
    VERIFY_WORLD("TransformEnd");
    if (!pushedTransforms.size()) {
        Error(
            "Unmatched pbrtTransformEnd() encountered. "
            "Ignoring it.");
        return;
    }
    curTransform = pushedTransforms.back();
    pushedTransforms.pop_back();
    activeTransformBits = pushedActiveTransformBits.back();
    pushedActiveTransformBits.pop_back();
}

void pbrtTexture(const std::string &name, const std::string &type,
                 const std::string &texname, const ParamSet &params) {
    VERIFY_WORLD("Texture");
    TextureParams tp(params, params, graphicsState.floatTextures,
                     graphicsState.spectrumTextures);
    if (type == "float") {
        // Create _Float_ texture and store in _floatTextures_
        if (graphicsState.floatTextures.find(name) !=
            graphicsState.floatTextures.end())
            Info("Texture \"%s\" being redefined", name.c_str());
        WARN_IF_ANIMATED_TRANSFORM("Texture");
        std::shared_ptr<Texture<Float>> ft =
            MakeFloatTexture(texname, curTransform[0], tp);
        if (ft) graphicsState.floatTextures[name] = ft;
    } else if (type == "color" || type == "spectrum") {
        // Create _color_ texture and store in _spectrumTextures_
        if (graphicsState.spectrumTextures.find(name) !=
            graphicsState.spectrumTextures.end())
            Info("Texture \"%s\" being redefined", name.c_str());
        WARN_IF_ANIMATED_TRANSFORM("Texture");
        std::shared_ptr<Texture<Spectrum>> st =
            MakeSpectrumTexture(texname, curTransform[0], tp);
        if (st) graphicsState.spectrumTextures[name] = st;
    } else
        Error("Texture type \"%s\" unknown.", type.c_str());
}

void pbrtMaterial(const std::string &name, const ParamSet &params) {
    VERIFY_WORLD("Material");
    graphicsState.material = name;
    graphicsState.materialParams = params;
    graphicsState.currentNamedMaterial = "";
}

void pbrtMakeNamedMaterial(const std::string &name, const ParamSet &params) {
    VERIFY_WORLD("MakeNamedMaterial");
    // error checking, warning if replace, what to use for transform?
    TextureParams mp(params, graphicsState.materialParams,
                     graphicsState.floatTextures,
                     graphicsState.spectrumTextures);
    std::string matName = mp.FindString("type");
    WARN_IF_ANIMATED_TRANSFORM("MakeNamedMaterial");
    if (matName == "")
        Error("No parameter string \"type\" found in MakeNamedMaterial");
    else {
        std::shared_ptr<Material> mtl = MakeMaterial(matName, mp);
        if (mtl) graphicsState.namedMaterials[name] = mtl;
    }
}

void pbrtNamedMaterial(const std::string &name) {
    VERIFY_WORLD("NamedMaterial");
    graphicsState.currentNamedMaterial = name;
}

void pbrtLightSource(const std::string &name, const ParamSet &params) {
    VERIFY_WORLD("LightSource");
    WARN_IF_ANIMATED_TRANSFORM("LightSource");
    MediumInterface mediumInterface = graphicsState.CreateMediumInterface();
    std::shared_ptr<Light> lt =
        MakeLight(name, params, curTransform[0], mediumInterface);
    if (lt == nullptr)
        Error("pbrtLightSource: light type \"%s\" unknown.", name.c_str());
    else
        renderOptions->lights.push_back(lt);
}

void pbrtAreaLightSource(const std::string &name, const ParamSet &params) {
    VERIFY_WORLD("AreaLightSource");
    graphicsState.areaLight = name;
    graphicsState.areaLightParams = params;
}

void pbrtShape(const std::string &name, const ParamSet &params) {
    VERIFY_WORLD("Shape");
    std::vector<std::shared_ptr<Primitive>> prims;
    std::vector<std::shared_ptr<AreaLight>> areaLights;
    if (!curTransform.IsAnimated()) {
        // Create primitives for static shape
        Transform *obj2world, *world2obj;
        transformCache.Lookup(curTransform[0], &obj2world, &world2obj);
        std::vector<std::shared_ptr<Shape>> shapes =
            MakeShapes(name, obj2world, world2obj,
                       graphicsState.reverseOrientation, params);
        if (shapes.size() == 0) return;
        std::shared_ptr<Material> mtl = graphicsState.CreateMaterial(params);
        params.ReportUnused();
        MediumInterface mediumInterface = graphicsState.CreateMediumInterface();
        for (auto s : shapes) {
            // Possibly create area light for shape
            std::shared_ptr<AreaLight> area;
            if (graphicsState.areaLight != "") {
                MediumInterface mediumInterface =
                    graphicsState.CreateMediumInterface();
                area = MakeAreaLight(graphicsState.areaLight, curTransform[0],
                                     mediumInterface,
                                     graphicsState.areaLightParams, s);
                areaLights.push_back(area);
            }
            prims.push_back(std::make_shared<GeometricPrimitive>(
                s, mtl, area, mediumInterface));
        }
    } else {
        // Create primitives for animated shape

        // Create initial _Shape_ for animated shape
        if (graphicsState.areaLight != "")
            Warning(
                "Ignoring currently set area light when creating "
                "animated shape");
        Transform *identity;
        transformCache.Lookup(Transform(), &identity, nullptr);
        std::vector<std::shared_ptr<Shape>> shapes = MakeShapes(
            name, identity, identity, graphicsState.reverseOrientation, params);
        if (shapes.size() == 0) return;
        std::shared_ptr<Material> mtl = graphicsState.CreateMaterial(params);
        params.ReportUnused();

        // Get _animatedObjectToWorld_ transform for shape
        Assert(MaxTransforms == 2);
        Transform *obj2world[2];
        transformCache.Lookup(curTransform[0], &obj2world[0], nullptr);
        transformCache.Lookup(curTransform[1], &obj2world[1], nullptr);
        AnimatedTransform animatedObjectToWorld(
            obj2world[0], renderOptions->transformStartTime, obj2world[1],
            renderOptions->transformEndTime);
        MediumInterface mediumInterface = graphicsState.CreateMediumInterface();
        for (auto s : shapes)
            prims.push_back(std::make_shared<GeometricPrimitive>(
                s, mtl, nullptr, mediumInterface));
        if (prims.size() > 1) {
            std::shared_ptr<Primitive> bvh = std::make_shared<BVHAccel>(prims);
            prims.clear();
            prims.push_back(bvh);
        }
        prims[0] = std::make_shared<TransformedPrimitive>(
            prims[0], animatedObjectToWorld);
    }
    // Add primitive to scene or current instance
    if (renderOptions->currentInstance) {
        if (areaLights.size())
            Warning("Area lights not supported with object instancing");
        renderOptions->currentInstance->insert(
            renderOptions->currentInstance->end(), prims.begin(), prims.end());
    } else {
        renderOptions->primitives.insert(renderOptions->primitives.end(),
                                         prims.begin(), prims.end());
        if (areaLights.size()) {
            renderOptions->lights.insert(renderOptions->lights.end(),
                                         areaLights.begin(), areaLights.end());
        }
    }
}

std::shared_ptr<Material> GraphicsState::CreateMaterial(
    const ParamSet &params) {
    TextureParams mp(params, materialParams, floatTextures, spectrumTextures);
    std::shared_ptr<Material> mtl;
    if (currentNamedMaterial != "" &&
        namedMaterials.find(currentNamedMaterial) != namedMaterials.end())
        mtl = namedMaterials[graphicsState.currentNamedMaterial];
    if (!mtl) mtl = MakeMaterial(material, mp);
    if (!mtl && material != "" && material != "none")
        mtl = MakeMaterial("matte", mp);
    return mtl;
}

MediumInterface GraphicsState::CreateMediumInterface() {
    MediumInterface m;
    if (currentInsideMedium != "") {
        if (renderOptions->namedMedia.find(currentInsideMedium) !=
            renderOptions->namedMedia.end())
            m.inside = renderOptions->namedMedia[currentInsideMedium].get();
        else
            Error("Named medium \"%s\" undefined.",
                  currentInsideMedium.c_str());
    }
    if (currentOutsideMedium != "") {
        if (renderOptions->namedMedia.find(currentOutsideMedium) !=
            renderOptions->namedMedia.end())
            m.outside = renderOptions->namedMedia[currentOutsideMedium].get();
        else
            Error("Named medium \"%s\" undefined.",
                  currentOutsideMedium.c_str());
    }
    return m;
}

void pbrtReverseOrientation() {
    VERIFY_WORLD("ReverseOrientation");
    graphicsState.reverseOrientation = !graphicsState.reverseOrientation;
}

void pbrtObjectBegin(const std::string &name) {
    VERIFY_WORLD("ObjectBegin");
    pbrtAttributeBegin();
    if (renderOptions->currentInstance)
        Error("ObjectBegin called inside of instance definition");
    renderOptions->instances[name] = std::vector<std::shared_ptr<Primitive>>();
    renderOptions->currentInstance = &renderOptions->instances[name];
}

STAT_COUNTER("Scene/Object instances created", nObjectInstancesCreated);
void pbrtObjectEnd() {
    VERIFY_WORLD("ObjectEnd");
    if (!renderOptions->currentInstance)
        Error("ObjectEnd called outside of instance definition");
    renderOptions->currentInstance = nullptr;
    pbrtAttributeEnd();
    ++nObjectInstancesCreated;
}

STAT_COUNTER("Scene/Object instances used", nObjectInstancesUsed);
void pbrtObjectInstance(const std::string &name) {
    VERIFY_WORLD("ObjectInstance");
    // Object instance error checking
    if (renderOptions->currentInstance) {
        Error("ObjectInstance can't be called inside instance definition");
        return;
    }
    if (renderOptions->instances.find(name) == renderOptions->instances.end()) {
        Error("Unable to find instance named \"%s\"", name.c_str());
        return;
    }
    std::vector<std::shared_ptr<Primitive>> &in =
        renderOptions->instances[name];
    if (in.size() == 0) return;
    ++nObjectInstancesUsed;
    if (in.size() > 1) {
        // Create aggregate for instance _Primitive_s
        std::shared_ptr<Primitive> accel(
            MakeAccelerator(renderOptions->AcceleratorName, in,
                            renderOptions->AcceleratorParams));
        if (!accel) accel = MakeAccelerator("bvh", in, ParamSet());
        Assert(accel != nullptr);
        in.erase(in.begin(), in.end());
        in.push_back(accel);
    }
    Assert(MaxTransforms == 2);
    Transform *instance2world[2];
    transformCache.Lookup(curTransform[0], &instance2world[0], nullptr);
    transformCache.Lookup(curTransform[1], &instance2world[1], nullptr);
    AnimatedTransform animatedInstanceToWorld(
        instance2world[0], renderOptions->transformStartTime, instance2world[1],
        renderOptions->transformEndTime);
    std::shared_ptr<Primitive> prim(
        std::make_shared<TransformedPrimitive>(in[0], animatedInstanceToWorld));
    renderOptions->primitives.push_back(prim);
}

void pbrtWorldEnd() {
    VERIFY_WORLD("WorldEnd");
    // Ensure there are no pushed graphics states
    while (pushedGraphicsStates.size()) {
        Warning("Missing end to pbrtAttributeBegin()");
        pushedGraphicsStates.pop_back();
        pushedTransforms.pop_back();
    }
    while (pushedTransforms.size()) {
        Warning("Missing end to pbrtTransformBegin()");
        pushedTransforms.pop_back();
    }

    // Create scene and render
    {
        std::unique_ptr<Integrator> integrator(renderOptions->MakeIntegrator());
        std::unique_ptr<Scene> scene(renderOptions->MakeScene());
        if (scene && integrator) integrator->Render(*scene);
        TerminateWorkerThreads();
    }

    // Clean up after rendering
    graphicsState = GraphicsState();
    transformCache.Clear();
    currentApiState = APIState::OptionsBlock;
    ReportThreadStats();
    if (PbrtOptions.quiet == false) {
        PrintStats(stdout);
        ReportProfilerResults(stdout);
    }
    for (int i = 0; i < MaxTransforms; ++i) curTransform[i] = Transform();
    activeTransformBits = AllTransformsBits;
    namedCoordinateSystems.erase(namedCoordinateSystems.begin(),
                                 namedCoordinateSystems.end());
    ImageTexture<Float, Float>::ClearCache();
    ImageTexture<RGBSpectrum, Spectrum>::ClearCache();
}

Scene *RenderOptions::MakeScene() {
    std::shared_ptr<Primitive> accelerator =
        MakeAccelerator(AcceleratorName, primitives, AcceleratorParams);
    if (!accelerator)
        accelerator = MakeAccelerator("bvh", primitives, ParamSet());
    Assert(accelerator != nullptr);
    Scene *scene = new Scene(accelerator, lights);
    // Erase primitives and lights from _RenderOptions_
    primitives.erase(primitives.begin(), primitives.end());
    lights.erase(lights.begin(), lights.end());
    return scene;
}

Integrator *RenderOptions::MakeIntegrator() const {
    std::shared_ptr<const Camera> camera(MakeCamera());

    std::shared_ptr<Sampler> sampler =
        MakeSampler(SamplerName, SamplerParams, camera->film);
    if (!sampler) {
        Error("Unable to create sampler.");
        return nullptr;
    }

    Integrator *integrator = nullptr;
    if (IntegratorName == "whitted")
        integrator = CreateWhittedIntegrator(IntegratorParams, sampler, camera);
    else if (IntegratorName == "directlighting")
        integrator =
            CreateDirectLightingIntegrator(IntegratorParams, sampler, camera);
    else if (IntegratorName == "path")
        integrator = CreatePathIntegrator(IntegratorParams, sampler, camera);
    else if (IntegratorName == "volpath")
        integrator = CreateVolPathIntegrator(IntegratorParams, sampler, camera);
    else if (IntegratorName == "bdpt") {
        integrator = CreateBDPTIntegrator(IntegratorParams, sampler, camera);
    } else if (IntegratorName == "mlt") {
        integrator = CreateMLTIntegrator(IntegratorParams, camera);
    } else if (IntegratorName == "sppm") {
        integrator = CreateSPPMIntegrator(IntegratorParams, camera);
    } else
        Warning("Integrator \"%s\" unknown.", IntegratorName.c_str());
    IntegratorParams.ReportUnused();
    // Warn if no light sources are defined
    if (lights.size() == 0)
        Warning(
            "No light sources defined in scene; "
            "possibly rendering a black image.");
    return integrator;
}

Camera *RenderOptions::MakeCamera() const {
    Filter *filter = MakeFilter(FilterName, FilterParams);
    Film *film = MakeFilm(FilmName, FilmParams, filter);
    if (!film) {
        Error("Unable to create film.");
        exit(1);
    }
    Camera *camera = ::MakeCamera(CameraName, CameraParams, CameraToWorld,
                                  renderOptions->transformStartTime,
                                  renderOptions->transformEndTime, film);
    if (!camera) {
        Error("Unable to create camera.");
        exit(1);
    }
    return camera;
}
