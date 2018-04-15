#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CAMERAS_HEMISPHERIC_H
#define PBRT_CAMERAS_HEMISPHERIC_H

// cameras/hemispheric.h*
#include "camera.h"
#include "film.h"
#include "film/intensityfilm.h"

namespace pbrt {

// ============================================================================

// HemisphericCamera Declarations
class HemisphericCamera : public Camera {

private:
    // Fields -----------------------------------------------------------------
    std::shared_ptr<IntensityFilm> nn_film = nullptr;

public:

    // Constructor ------------------------------------------------------------
    HemisphericCamera(
            const AnimatedTransform &CameraToWorld,
            Float shutterOpen,
            Float shutterClose,
            Film* film,
            const Medium* medium
            ) :
        Camera(CameraToWorld, shutterOpen, shutterClose, film, medium) {}

    // Public methods =========================================================

    Float GenerateRay(
            const CameraSample &sample,
            Ray *
            ) const;

    Spectrum getLightSample(
            int x,
            int y,
            Vector3f* wi
            );

    void set_nn_film(
            std::shared_ptr<IntensityFilm> nn_film
            );

    Spectrum get_light_sample_nn(
            int x,
            int y,
            Vector3f* wi
            );

};

// ============================================================================

HemisphericCamera* CreateHemisphericCamera(
        int xres,
        int yres,
        const Medium *medium,
        Point3f pos,
        Point3f dir,
        Point2i originalPixel,
        std::string output_file_name
        );


} // namespace pbrt


#endif // HEMISPHERIC_H
