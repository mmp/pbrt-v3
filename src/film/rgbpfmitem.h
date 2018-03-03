#ifndef RGBPFMITEM_H
#define RGBPFMITEM_H

namespace pbrt {

// Implementation of PfmItem for a single scalar value
class RgbPfmItem : PfmItem {

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

};

} // namespace pbrt

#endif // RGBPFMITEM_H
