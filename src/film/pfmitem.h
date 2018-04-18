#ifndef PFMITEM_H
#define PFMITEM_H

#include <iostream>
#include <cstdlib>

#include "pbrt.h"
#include "spectrum.h"

namespace pbrt {

class PfmItem {

private:

    // Fields -----------------------------------------------------------------
    float r = 0.0;
    float g = 0.0;
    float b = 0.0;
    bool rgb = false;

public:

    // Constructor ------------------------------------------------------------

    PfmItem(float r, float g, float b)
    {
        this->r = r;
        this->g = g;
        this->b = b;
        this->rgb = true;
    }

    PfmItem(float v)
    {
        this->r = r;
        this->rgb = false;
    }

    // Either 1 or 3
    int get_number_components() {
        if (rgb) {
            return 3;
        } else {
            return 1;
        }
    }

    // Get scalar component
    float get_single_component() {
        return r;
    }

    // Get triple component
    void get_triple_component(float &rr, float &gg, float &bb) {
        rr = r;
        gg = g;
        bb = b;
    }

    // Scalar multiply
    PfmItem scalar_multiply(float c) {
        if (rgb) {
            return PfmItem(r*c, g*c, b*c);
        } else {
            return PfmItem(r*c);
        }
    }

    // Get as spectrum
    Spectrum as_spectrum() {
        Float rgb[3];
        get_triple_component(rgb[0], rgb[1], rgb[2]);
        return Spectrum::FromRGB(rgb);
    }

    float magnitude() {
        return r + g + b;
    }

};

}

#endif // PFMITEM_H
