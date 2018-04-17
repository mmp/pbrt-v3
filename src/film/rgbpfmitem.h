#ifndef RGBPFMITEM_H
#define RGBPFMITEM_H

#include "pbrt.h"
#include "spectrum.h"

namespace pbrt {

// Implementation of PfmItem for a single scalar value
class RgbPfmItem : public PfmItem {

private:
    float r = 0.0;
    float g = 0.0;
    float b = 0.0;

public:

    RgbPfmItem(float r, float g, float b) :
        r(r),
        g(g),
        b(b)
    {

    }

    virtual int get_number_components() {
        return 3;
    }

    virtual float get_single_component() {
        std::cerr << "rgbpfmitem.h: Error, trying to get 1 component, but a RgbPfmItem has 3 components" << std::endl;
        exit(1);
    }

    virtual void get_triple_component(float &rr, float &gg, float &bb) {
        rr = r;
        gg = g;
        bb = b;
    }

    virtual std::unique_ptr<PfmItem> scalar_multiply(float c) {
        std::unique_ptr<RgbPfmItem> res (
                    new RgbPfmItem(
                        r * c,
                        g * c,
                        b * c
                        )
                    );
        return res;
    }

    virtual Spectrum as_spectrum() {
        Float rgb[3];
        get_triple_component(rgb[0], rgb[1], rgb[2]);
        return Spectrum::FromRGB(
                    rgb
                    );
    }

    virtual std::unique_ptr<PfmItem> clone() {
        std::unique_ptr<PfmItem> res (
                    new RgbPfmItem(r, g, b)
                    );
        return res;
    }

};

} // namespace pbrt

#endif // RGBPFMITEM_H
