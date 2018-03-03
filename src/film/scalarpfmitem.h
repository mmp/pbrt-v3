#ifndef SCALARPFMITEM_H
#define SCALARPFMITEM_H

namespace pbrt {

// Implementation of PfmItem for a single scalar value
class ScalarPfmItem : PfmItem {

private:
    float value = 0.0;

public:

    ScalarPfmItem(value) :
        value(value)
    {

    }

    virtual int get_number_components() {
        return 1;
    }

    virtual float get_single_component() {
        return value;
    }

    virtual void get_triple_component(float &r, float &g, float &b) {
        std::cerr << "scalarpfmitem.h: Error, trying to get 3 components, but a ScalarPfmItem only has 1 component" << std::endl;
        exit(1);
    }

};

} // namespace pbrt

#endif // SCALARPFMITEM_H
