#pragma once

#include "utils/vec.h"
#include "utils/ray.h"

namespace sc {

template<typename NumericT>
struct PixelSample
{
    std::size_t pixelY;
    std::size_t pixelX;
    utils::Vec<NumericT, 2> uv;
    utils::Ray<NumericT, 3> ray;
};

} // namespace sc

namespace sc::internal
{

template<typename NumericT>
class CameraView
{
public:
    using Vec3 = utils::Vec<NumericT, 3>;

    Vec3 origin;
    Vec3 forward;
    Vec3 right;
    Vec3 up;

    Vec3 pixelOrigin;
    Vec3 stepX;
    Vec3 stepY;

    std::size_t width  = 0;
    std::size_t height = 0;

    class Iterator;
    Iterator begin() const { return Iterator(this, 0); }
    Iterator end() const { return Iterator(this, width * height); }

    PixelSample<NumericT> getPixelSample(std::size_t x, std::size_t y) const
    {
        PixelSample<NumericT> sample;

        sample.pixelX = x;
        sample.pixelY = y;

        const NumericT uvStepX = NumericT(2) / static_cast<NumericT>(width);
        sample.uv[0] = NumericT(-1) + NumericT(x) * uvStepX;

        sample.uv[1] =
            (NumericT(y) - NumericT(height) * NumericT(0.5))
            / (NumericT(height) * NumericT(0.5));

        sample.ray.pos() = origin;

        auto rayRot = pixelOrigin;
        rayRot = binaryOp(rayRot,
                          scalarBinaryOp(stepY, NumericT(y), std::multiplies<>{}),
                          std::plus<>{});

        rayRot = binaryOp(rayRot,
                          scalarBinaryOp(stepX, NumericT(x), std::multiplies<>{}),
                          std::plus<>{});

        sample.ray.rot() = rayRot;

        return sample;
    }
};

} // namespace sc::internal