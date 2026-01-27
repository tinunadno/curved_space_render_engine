#pragma once

#include "utils/vec.h"
#include "camera/camera_view_iterator.h"

namespace mrc
{

template<typename NumericT>
NumericT getRayPolyCrossT(
    const sc::utils::Vec<NumericT,3>& rayOrig,
    const sc::utils::Vec<NumericT,3>& rayDir,
    const sc::utils::Vec<NumericT,3>& p1,
    const sc::utils::Vec<NumericT,3>& p2,
    const sc::utils::Vec<NumericT,3>& p3)
{

    const NumericT abx = p2[0] - p1[0];
    const NumericT aby = p2[1] - p1[1];
    const NumericT abz = p2[2] - p1[2];

    const NumericT acx = p3[0] - p1[0];
    const NumericT acy = p3[1] - p1[1];
    const NumericT acz = p3[2] - p1[2];
    
    const NumericT nx = aby * acz - abz * acy;
    const NumericT ny = abz * acx - abx * acz;
    const NumericT nz = abx * acy - aby * acx;

    const NumericT denom =
        nx * rayDir[0] +
        ny * rayDir[1] +
        nz * rayDir[2];

    if (denom == NumericT(0))
        return std::numeric_limits<NumericT>::infinity();

    const NumericT dx = p1[0] - rayOrig[0];
    const NumericT dy = p1[1] - rayOrig[1];
    const NumericT dz = p1[2] - rayOrig[2];

    const NumericT numer =
        nx * dx +
        ny * dy +
        nz * dz;

    const NumericT t = numer / denom;

    return t;
}

template<typename NumericT>
NumericT getDistanceFromRayToPoly(
    sc::PixelSample<NumericT> ps,
    const sc::utils::Vec<NumericT,3>& p1,
    const sc::utils::Vec<NumericT,3>& p2,
    const sc::utils::Vec<NumericT,3>& p3)
{
    const auto rayOrig = ps.ray.pos();
    const auto rayDir = ps.ray.rot();
    NumericT t = getRayPolyCrossT(rayOrig, rayDir, p1, p2, p3);
    auto crossPnt = rayOrig + (rayDir * t);
    return sc::utils::distance(crossPnt, rayDir);
}

} 