#ifndef PFMITEM_H
#define PFMITEM_H

namespace pbrt {

// Interface
class PfmItem {

public:
    // Either 1 or 3
    virtual int get_number_components() = 0;

    // Get scalar component
    virtual double get_single_component() = 0;

    // Get triple component
    virtual void get_triple_component(int* r, int* g, int* b) = 0;

};

}

#endif // PFMITEM_H
