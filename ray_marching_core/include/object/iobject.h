#pragma once

#include <memory>

#include "utils/vec.h"
#include "shading/material.h"

namespace rmc::object
{

template<typename NumericT>
class IObject
{
public:
    virtual ~IObject() = default;

    virtual NumericT sdf(const sc::utils::Vec<NumericT, 3>& vec) const = 0;
    virtual sc::utils::Vec<NumericT, 3> normal(const sc::utils::Vec<NumericT, 3>& vec) const = 0;

};

template<typename NumericT>
struct SceneObject
{
    std::shared_ptr<IObject<NumericT>> shape;
    shader::Material material;
};

} // namespace rmc::object