#pragma once

#include "entry_point.h"
#include "utils/mat.h"
#include "camera/camera.h"

namespace mrc
{

namespace internal
{

template <typename NumericT>
struct ProjectedVertex {
        sc::utils::Vec<NumericT, 2> pixel; // screen space coordinates
        NumericT invW;                     // 1/W (for z interpolation)
        NumericT depth;                    // Z_view
    };
} // namespace internal

template <typename NumericT>
sc::utils::Vec<NumericT, 3> worldUp{0, 1, 0};


template<typename NumericT>
sc::utils::Vec<NumericT, 3> getForward(const sc::utils::Vec<NumericT, 3>& rotation)
{
    const NumericT pitch = rotation[0];
    const NumericT yaw   = rotation[1];

    const NumericT cx = std::cos(pitch);
    const NumericT sx = std::sin(pitch);
    const NumericT cy = std::cos(yaw);
    const NumericT sy = std::sin(yaw);
    return sc::utils::norm(sc::utils::Vec<NumericT, 3>
    {
        cx * sy,
        sx,
       -cx * cy
    });
}

template <typename NumericT>
sc::utils::Vec<NumericT, 3> getRight(const sc::utils::Vec<NumericT, 3>& forward)
{
    return sc::utils::norm(sc::utils::cross(forward, worldUp<NumericT>));
}

template <typename NumericT>
sc::utils::Vec<NumericT, 3> getUp(const sc::utils::Vec<NumericT, 3>& forward,
    const sc::utils::Vec<NumericT, 3>& right)
{
    return sc::utils::cross(right, forward);
}

template <typename NumericT>
sc::utils::Mat<NumericT, 4, 4>
getViewMatrix(const sc::Camera<NumericT, sc::VecArray>& cam)
{
    using Mat4 = sc::utils::Mat<NumericT, 4, 4>;
    using Vec3 = sc::utils::Vec<NumericT, 3>;

    Vec3 forward = getForward(cam.rot());
    Vec3 right = getRight(forward);
    Vec3 up = getUp(forward, right);

    Mat4 View{
        right[0],    right[1],    right[2],   -sc::utils::dot(right, cam.pos()),
        up[0],       up[1],       up[2],      -sc::utils::dot(up, cam.pos()),
       -forward[0], -forward[1], -forward[2],  sc::utils::dot(forward, cam.pos()),
        0,           0,           0,           1
    };

    return View;
}

template <typename NumericT>
sc::utils::Mat<NumericT, 4, 4> getProjectionMatrix(const sc::Camera<NumericT, sc::VecArray>& cam) {
    using Mat4 = sc::utils::Mat<NumericT, 4, 4>;
    const NumericT fx = cam.len() / cam.size()[0];
    const NumericT fy = cam.len() / cam.size()[1];

    const NumericT near = NumericT(0.01);
    const NumericT far  = NumericT(1000.0);

    const NumericT A = (far + near) / (near - far);
    const NumericT B = (2 * far * near) / (near - far);

    Mat4 Proj{
        fx, 0,  0,  0,
        0, fy,  0,  0,
        0,  0,  A,  B,
        0,  0, -1,  0
    };
    return Proj;
}

template <typename NumericT>
internal::ProjectedVertex<NumericT> projectVertex(
    const sc::utils::Vec<NumericT, 3>& ws,
    const sc::utils::Mat<NumericT, 4, 4>& viewProj,
    const sc::Camera<NumericT, sc::VecArray>& cam)
{
    auto clip = viewProj * sc::utils::Vec<NumericT, 4>{ws[0], ws[1], ws[2], 1.0f};
    NumericT invW = 1.0f / clip[3];
    sc::utils::Vec<NumericT, 3> ndc {
        clip[0] * invW,
        clip[1] * invW,
        clip[2] * invW
    };

    sc::utils::Vec<NumericT, 2> pixel {
        (ndc[0] + 1.0f) * 0.5f * cam.res()[0],
        (1.0f - ndc[1]) * 0.5f * cam.res()[1]
    };

    return {pixel, invW, clip[2]}; // clip[2] is a depth
}

} // namespace mrc