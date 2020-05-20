/*************************************************************************
    > File Name: src/accelerators/lighttree.h
    > Author: Yuchen Wang
    > Mail: wyc8094@gmail.com 
    > Created Time: Wed May 20 15:14:36 2020
 ************************************************************************/

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_ACCELERATORS_LIGHTTREE_H_
#define PBRT_ACCELERATORS_LIGHTTREE_H_

#include "light.h"
#include "pbrt.h"

#include <memory>

namespace pbrt {

class LightTree {
  public:
    enum class SplitMethod {
      Middle,
      EqualCounts,
      SAOH
    };

  public:
    LightTree(std::vector<std::shared_ptr<Light>> lights,
        int maxLightsPerNode = 1,
        SplitMethod splitMethod = SplitMethod::SAOH);
    ~LightTree();

  private:
    std::vector<std::shared_ptr<Light>> _light;
    const int _maxLightsPerNode;
    const SplitMethod _splitMethod;
};

std::shared_ptr<LightTree> CreateLightTree(
    std::vector<std::shared_ptr<Light>> lights,
    const ParamSet& params
    );

} // namespace pbrt

#endif // PBRT_ACCELERATORS_LIGHTTREE_H_
