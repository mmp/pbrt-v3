#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CAMERAS_HEMISPHERIC_H
#define PBRT_CAMERAS_HEMISPHERIC_H

// cameras/hemispheric.h*
#include "camera.h"
#include "film.h"

namespace pbrt {

// HemisphericCamera Declarations
class HemisphericCamera : public Camera {

public:

    HemisphericCamera(
            const AnimatedTransform &CameraToWorld,
            Float shutterOpen,
            Float shutterClose,
            Film* film,
            const Medium* medium
            ) :
        Camera(CameraToWorld, shutterOpen, shutterClose, film, medium) {}

    Float GenerateRay(
            const CameraSample &sample,
            Ray *
            ) const;

};


HemisphericCamera* CreateHemisphericCamera(
        int xres,
        int yres,
        const Medium *medium,
        Point3f pos,
        Point3f dir,
        Point2i originalPixel
        );


} // namespace pbrt


#endif // HEMISPHERIC_H
