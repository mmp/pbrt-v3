#ifndef IISPTRNG_H
#define IISPTRNG_H

#include <memory>
#include "rng.h"

namespace pbrt {

class IisptRng
{
private:
    // Fields -----------------------------------------------------------------

    std::unique_ptr<RNG> rng;


public:
    // Constructor ------------------------------------------------------------

    IisptRng(uint64_t seed);

    // Public methods ---------------------------------------------------------

    // Upper bound not inclusive
    // Lower bound 0 inclusive
    uint32_t uniform_uint32(uint32_t bound);

    // Returns TRUE with <prob> probability
    bool bool_probability(float prob);

    float uniform_float();
};

}

#endif // IISPTRNG_H
