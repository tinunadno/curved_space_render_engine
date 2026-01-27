#pragma once

#include "entry_point.h"
#include "utils/mat.h"
#include "camera/camera.h"

namespace mrc {

template <typename NumericT>
sc::utils::Mat<NumericT, 4, 4>
makeProjectionMatrixFromCamera(const sc::Camera<NumericT, sc::VecArray>& cam)
{
    using Mat4 = sc::utils::Mat<NumericT, 4, 4>;
    using Mat3 = sc::utils::Mat<NumericT, 3, 3>;
    using Vec3 = sc::utils::Vec<NumericT, 3>;

    const NumericT cx = std::cos(cam.rot()[0]);
    const NumericT sx = std::sin(cam.rot()[0]);
    const NumericT cy = std::cos(cam.rot()[1]);
    const NumericT sy = std::sin(cam.rot()[1]);
    const NumericT cz = std::cos(cam.rot()[2]);
    const NumericT sz = std::sin(cam.rot()[2]);

    Mat3 Rx{
        1,  0,   0,
        0,  cx, -sx,
        0,  sx,  cx
    };

    Mat3 Ry{
        cy, 0, sy,
         0, 1,  0,
       -sy, 0, cy
   };

    Mat3 Rz{
        cz, -sz, 0,
        sz,  cz, 0,
         0,   0, 1
    };

    Mat3 R = Rz * Ry * Rx;

    Mat4 View{
        R(0,0), R(1,0), R(2,0), 0,
        R(0,1), R(1,1), R(2,1), 0,
        R(0,2), R(1,2), R(2,2), 0,
        0,      0,      0,      1
    };

    Vec3 t = -(R * cam.pos());

    View(0,3) = t[0];
    View(1,3) = t[1];
    View(2,3) = t[2];

    const NumericT fx = cam.len() / cam.size()[0];
    const NumericT fy = cam.len() / cam.size()[1];

    const NumericT near = NumericT(0.01);
    const NumericT far  = NumericT(1000.0);

    const NumericT A = (far + near) / (near - far);
    const NumericT B = (2 * far * near) / (near - far);

    Mat4 Projection{
        fx, 0,  0,  0,
        0, fy,  0,  0,
        0,  0,  A,  B,
        0,  0, -1,  0
    };

    return Projection * View;
}

template <typename NumericT>
sc::utils::Vec<NumericT, 2> ndcToPixel(const sc::utils::Vec<NumericT, 3>& ndc,
    const sc::Camera<NumericT, sc::VecArray> cam)
{
    return sc::utils::Vec<NumericT, 2>{
        (ndc[0] + NumericT(1)) * NumericT(0.5) * cam.res()[0],
        (NumericT(1) - ndc[1]) * NumericT(0.5) * cam.res()[1]
    };
}


template <typename NumericT>
sc::utils::Vec<NumericT, 2> clipToPixel(
    const sc::utils::Vec<NumericT, 3>& clip,
    const sc::Camera<NumericT, sc::VecArray> cam)
{
    // behind the camera
    if (clip[3] <= 0)
        return sc::utils::Vec<NumericT, 2>{-1, -1};
    sc::utils::Vec<NumericT, 3> ndc{
        clip[0] / clip[3],
        clip[1] / clip[3],
        clip[2] / clip[3]
    };

    return ndcToPixel(ndc, cam);
}
template <typename NumericT>
sc::utils::Vec<NumericT, 2> wsToPixel(
    const sc::utils::Vec<NumericT, 3>& ws,
    const sc::utils::Mat<NumericT, 4, 4>& proj,
    const sc::Camera<NumericT, sc::VecArray> cam)
{
    sc::utils::Vec clip4{proj * sc::utils::Vec<NumericT, 4>{ws[0], ws[1], ws[2], NumericT(1.)}};
    sc::utils::Vec<NumericT, 3> clip{clip4[0], clip4[1], clip4[2]};
    return clipToPixel(clip, cam);
}

} // namespace mrc