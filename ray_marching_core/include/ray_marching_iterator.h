#pragma once

#include "ray_marching_params.h"
#include "ray_marching_result.h"
#include "scene.h"
#include "utils/ray.h"

namespace rmc
{

template<typename NumericT>
RayMarchingResult<NumericT> marchRay(
    const sc::utils::Ray<NumericT, 3>& ray,
    Scene<NumericT>& scene,
    const RayMarchingParams<NumericT>& params = {NumericT(1e-6, 30)})
{
    RayMarchingResult<NumericT> ret;
    internal::SdfResult<NumericT> sdfResult;
    NumericT passedDistance = 1.f;
    for (int i = 0; i < params.maxIterations; i++)
    {
        scene.sdf(ray.pos() + ray.rot() * passedDistance, sdfResult);
        passedDistance += sdfResult.distance;
        if (sdfResult.distance <= params.threshold)
            goto succeed;
    }
    ret.reachedThreshold = false;
    return ret;
succeed:
    ret.finalPosition = ray.pos() + ray.rot() * passedDistance;
    ret.normal = sdfResult.closestObject.normal();
    ret.distance = sdfResult.distance;
    ret.reachedThreshold = true;
    return ret;
}

} // namespace rmc