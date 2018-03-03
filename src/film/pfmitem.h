#ifndef PFMITEM_H
#define PFMITEM_H

#include <iostream>
#include <cstdlib>

namespace pbrt {

// Interface
class PfmItem {

public:
    // Either 1 or 3
    virtual int get_number_components() {
        std::cerr << "pfmitem.h: get_number_components() calling method on interface" << std::endl;
        exit(1);
        return 0;
    }

    // Get scalar component
    virtual float get_single_component() {
        std::cerr << "pfmitem.h: get_single_component() calling method on interface" << std::endl;
        exit(1);
        return 0.0;
    }

    // Get triple component
    virtual void get_triple_component(float &r, float &g, float &b) {
        std::cerr << "pfmitem.h: get_triple_component() calling method on interface" << std::endl;
        exit(1);
    }

};

}

#endif // PFMITEM_H
