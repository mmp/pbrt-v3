#ifndef PFMITEM_H
#define PFMITEM_H

namespace pbrt {

// Interface
class PfmItem {

public:
    // Either 1 or 3
    virtual int get_number_components() = 0;

    // Get scalar component
    virtual float get_single_component() = 0;

    // Get triple component
    virtual void get_triple_component(float &r, float &g, float &b) = 0;

};

}

#endif // PFMITEM_H
