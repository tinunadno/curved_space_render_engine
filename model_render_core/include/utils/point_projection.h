#pragma once

#include "entry_point.h"
#include "utils/mat.h"
#include "camera/camera.h"
#include "light_source.h"

namespace mrc
{

template<typename NumericT>
struct FragmentInput {
    sc::utils::Vec<NumericT, 2> uv;
    sc::utils::Vec<NumericT, 3> normal;
    sc::utils::Vec<NumericT, 3> worldPos;
    sc::utils::Vec<NumericT, 3> cameraPos;
    sc::utils::Vec<NumericT, 3> tangent;
    sc::utils::Vec<NumericT, 3> bitangent;
    NumericT depth;
    const std::vector<LightSource<NumericT>>& lights;
};



namespace internal
{

/// Per-vertex attributes that flow through the pipeline.
template<typename NumericT>
struct VertexAttributes {
    sc::utils::Vec<NumericT, 2> uv{};
    sc::utils::Vec<NumericT, 3> normal{};
    sc::utils::Vec<NumericT, 3> worldPos{};
    sc::utils::Vec<NumericT, 3> tangent{};
    sc::utils::Vec<NumericT, 3> bitangent{};

    VertexAttributes lerp(const VertexAttributes& o, NumericT t) const {
        return VertexAttributes{
            uv        + (o.uv        - uv)        * t,
            normal    + (o.normal    - normal)     * t,
            worldPos  + (o.worldPos  - worldPos)   * t,
            tangent   + (o.tangent   - tangent)    * t,
            bitangent + (o.bitangent - bitangent)  * t
        };
    }

    VertexAttributes operator*(NumericT s) const {
        return VertexAttributes{uv * s, normal * s, worldPos * s, tangent * s, bitangent * s};
    }

    VertexAttributes operator+(const VertexAttributes& o) const {
        return VertexAttributes{
            uv + o.uv, normal + o.normal, worldPos + o.worldPos,
            tangent + o.tangent, bitangent + o.bitangent
        };
    }
};

template <typename NumericT>
struct ProjectedVertex {
    sc::utils::Vec<NumericT, 2> pixel;
    NumericT invW;
    NumericT depth;
    VertexAttributes<NumericT> attr;
};

template <typename NumericT>
struct ClipVertex {
    sc::utils::Vec<NumericT, 4> clip;
    NumericT invW;
    VertexAttributes<NumericT> attr;
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
    return sc::utils::norm(sc::utils::Vec<NumericT, 3>{
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
internal::ClipVertex<NumericT> wsToClip(
    const sc::utils::Vec<NumericT, 3>& ws,
    const sc::utils::Mat<NumericT, 4, 4>& viewProj,
    const internal::VertexAttributes<NumericT>& attr = {})
{
    auto clip = viewProj * sc::utils::Vec<NumericT, 4>{ws[0], ws[1], ws[2], NumericT(1)};
    NumericT invW = NumericT(1) / clip[3];
    return {clip, invW, attr};
}

template <typename NumericT>
internal::ProjectedVertex<NumericT> clipToProjectedVertex(
    const internal::ClipVertex<NumericT>& clip,
    const sc::Camera<NumericT, sc::VecArray>& cam)
{
    sc::utils::Vec<NumericT, 3> ndc {
        clip.clip[0] * clip.invW,
        clip.clip[1] * clip.invW,
        clip.clip[2] * clip.invW
    };

    sc::utils::Vec<NumericT, 2> pixel {
        (ndc[0] + NumericT(1)) * NumericT(0.5) * cam.res()[0],
        (NumericT(1) - ndc[1]) * NumericT(0.5) * cam.res()[1]
    };

    return {pixel, clip.invW, ndc[2], clip.attr};
}

template <typename NumericT>
internal::ProjectedVertex<NumericT> projectVertex(
    const sc::utils::Vec<NumericT, 3>& ws,
    const sc::utils::Mat<NumericT, 4, 4>& viewProj,
    const sc::Camera<NumericT, sc::VecArray>& cam)
{
    return clipToProjectedVertex(wsToClip(ws, viewProj), cam);
}

} // namespace mrc
