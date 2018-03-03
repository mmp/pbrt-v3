#ifndef NULLPFMITEM_H
#define NULLPFMITEM_H

#include "pfmitem.h"

namespace pbrt {

// Simple default null-implementation of a PfmItem
class NullPfmItem : public PfmItem {

public:

    NullPfmItem() {

    }

    virtual int get_number_components() {
        return 1;
    }

    virtual float get_single_component() {
        return 0.0;
    }

    virtual void get_triple_component(float &r, float &g, float &b) {
        r = 0;
        g = 0;
        b = 0;
    }

};

} // namespace pbrt

#endif // NULLPFMITEM_H
