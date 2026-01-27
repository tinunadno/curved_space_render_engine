#pragma once

#include "utils/vec.h"
#include "model.h"

namespace mrc
{

template<typename NumericT>
sc::utils::Vec<NumericT, 3>
getFaceNormal(const Model<NumericT>& model, std::size_t faceIdx)
{
    const auto& n1 = model.normals()[model.faces()[faceIdx][0][2]];
    const auto& n2 = model.normals()[model.faces()[faceIdx][1][2]];
    const auto& n3 = model.normals()[model.faces()[faceIdx][2][2]];
    sc::utils::Vec<NumericT, 3> faceNormal = (n1 + n2 + n3) / NumericT(3);
    return faceNormal;
}

}