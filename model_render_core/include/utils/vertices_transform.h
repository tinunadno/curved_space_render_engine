#pragma once
#include <vector>

#include "utils/vec.h"

namespace mrc::internal
{
template <typename NumericT>
std::vector<sc::utils::Vec<NumericT, 3>> transformVerticies(
    const std::vector<sc::utils::Vec<NumericT, 3>>& verticies,
    const sc::utils::Vec<NumericT, 3>& pos,
    const sc::utils::Vec<NumericT, 3>& rot)
{
    std::vector<sc::utils::Vec<NumericT, 3>> ret;
    ret.reserve(verticies.size());
    for (const auto& v : verticies)
    {
        ret.emplace_back(rotateEuler(v, rot) + pos);
    }
    return ret;
}
} // namespace mrc::internal