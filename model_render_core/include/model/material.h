#pragma once
#include "texture.h"
#include <memory>

namespace mrc
{

/// Material properties for a model.
/// Uses shared_ptr for textures so that models sharing the same material
/// can reference the same texture data without copying.  Textures are
/// immutable after creation, so sharing is always safe.
template<typename NumericT>
struct Material {
    sc::utils::Vec<NumericT, 3> baseColor{1, 1, 1};
    std::shared_ptr<Texture<NumericT>> diffuseMap  = nullptr;
    std::shared_ptr<Texture<NumericT>> normalMap   = nullptr;
    NumericT ambient   = 0.1;
    NumericT specular  = 0.0;
    NumericT shininess = 32.0;
};

} // namespace mrc
