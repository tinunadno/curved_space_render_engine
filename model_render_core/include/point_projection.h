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
sc::utils::Mat<NumericT, 4, 4> getViewMatrix(const sc::Camera<NumericT, sc::VecArray>& cam) {

    using Mat4 = sc::utils::Mat<NumericT, 4, 4>;
    using Mat3 = sc::utils::Mat<NumericT, 3, 3>;
    using Vec3 = sc::utils::Vec<NumericT, 3>;

    const NumericT cx = std::cos(cam.rot()[0]), sx = std::sin(cam.rot()[0]);
    const NumericT cy = std::cos(cam.rot()[1]), sy = std::sin(cam.rot()[1]);
    const NumericT cz = std::cos(cam.rot()[2]), sz = std::sin(cam.rot()[2]);

    Mat3 Rx{1, 0, 0, 0, cx, -sx, 0, sx, cx};
    Mat3 Ry{cy, 0, sy, 0, 1,  0, -sy, 0, cy};
    Mat3 Rz{cz, -sz, 0, sz,  cz, 0, 0, 0, 1};

    Mat3 R = Rz * Ry * Rx;

    Mat4 View{
        R(0,0), R(1,0), R(2,0), 0,
        R(0,1), R(1,1), R(2,1), 0,
        R(0,2), R(1,2), R(2,2), 0,
        0,          0,          0,          1
    };

    Vec3 t = -(R * cam.pos());
    View(0,3) = t[0];
    View(1,3) = t[1];
    View(2,3) = t[2];
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