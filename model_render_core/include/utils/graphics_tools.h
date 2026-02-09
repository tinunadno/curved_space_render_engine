#pragma once
#include "glfw_render.h"
#include "point_projection.h"
#include "scene_cache.h"

namespace mrc::gt
{

template<typename NumericT>
void drawLine(
    sc::GLFWRenderer& renderer,
    const sc::utils::Vec<NumericT, 2>& a,
    const sc::utils::Vec<NumericT, 2>& b,
    const sc::utils::Vec<NumericT, 3>& color)
{
    int x0 = static_cast<int>(a[0]);
    int y0 = static_cast<int>(a[1]);
    int x1 = static_cast<int>(b[0]);
    int y1 = static_cast<int>(b[1]);

    const int dx = std::abs(x1 - x0);
    const int dy = std::abs(y1 - y0);

    const int sx = (x0 < x1) ? 1 : -1;
    const int sy = (y0 < y1) ? 1 : -1;

    int err = dx - dy;

    while (true)
    {
        if (x0 >= 0 && y0 >= 0 &&
            x0 < static_cast<int>(renderer.getRenderWidth()) &&
            y0 < static_cast<int>(renderer.getRenderHeight()))
        {
            renderer.setPixel(x0, y0, color);
        }

        if (x0 == x1 && y0 == y1)
            break;

        const int e2 = err << 1;

        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

template<typename NumericT>
NumericT edgeFunction(
    const sc::utils::Vec<NumericT, 2>& a,
    const sc::utils::Vec<NumericT, 2>& b,
    const sc::utils::Vec<NumericT, 2>& p)
{
    return (p[0] - a[0]) * (b[1] - a[1]) -
           (p[1] - a[1]) * (b[0] - a[0]);
}

template<typename NumericT>
void drawTriangle(
    sc::GLFWRenderer& renderer,
    const sc::utils::Vec<NumericT, 2>& v0,
    const sc::utils::Vec<NumericT, 2>& v1,
    const sc::utils::Vec<NumericT, 2>& v2,
    const sc::utils::Vec<NumericT, 3>& color)
{
    int tmp = std::min({v0[0], v1[0], v2[0]});
    const int minX = std::max(0, tmp);

    tmp = std::max({v0[0], v1[0], v2[0]});
    const int maxX = std::min(static_cast<int>(renderer.getRenderWidth()) - 1, tmp);

    tmp = std::min({v0[1], v1[1], v2[1]});
    const int minY = std::max(0, tmp);

    tmp = std::max({v0[1], v1[1], v2[1]});
    const int maxY = std::min(static_cast<int>(renderer.getRenderHeight()) - 1, tmp);

    const int area = edgeFunction(v0, v1, v2);
    if (area == 0)
        return;

    for (int y = minY; y <= maxY; ++y)
    {
        for (int x = minX; x <= maxX; ++x)
        {
            sc::utils::Vec<NumericT, 2> p{static_cast<NumericT>(x),
                                           static_cast<NumericT>(y)};

            int w0 = edgeFunction(v1, v2, p);
            int w1 = edgeFunction(v2, v0, p);
            int w2 = edgeFunction(v0, v1, p);

            if ((w0 >= 0 && w1 >= 0 && w2 >= 0 && area > 0) ||
                (w0 <= 0 && w1 <= 0 && w2 <= 0 && area < 0))
            {
                renderer.setPixel(x, y, color);
            }
        }
    }
}

template<typename NumericT>
internal::ClipVertex<NumericT> findClipCross(
    const internal::ClipVertex<NumericT>& a,
    const internal::ClipVertex<NumericT>& b)
{
    NumericT da = a.clip[2] + a.clip[3];
    NumericT db = b.clip[2] + b.clip[3];
    NumericT t = da / (da - db);

    internal::ClipVertex<NumericT> r;
    r.clip = a.clip + (b.clip - a.clip) * t;
    r.invW = NumericT(1) / r.clip[3];
    r.attr = a.attr.lerp(b.attr, t);
    return r;
}


template<typename NumericT>
std::array<internal::ProjectedVertex<NumericT>, 3> handleTwoClipOut(
    const internal::ClipVertex<NumericT>& out1,
    const internal::ClipVertex<NumericT>& out2,
    const internal::ClipVertex<NumericT>& in,
    const sc::Camera<NumericT, sc::VecArray>& cam)
{
    return {
        clipToProjectedVertex(findClipCross(out1, in), cam),
        clipToProjectedVertex(findClipCross(out2, in), cam),
        clipToProjectedVertex(in, cam),
    };
}


template<typename NumericT>
std::pair<std::array<internal::ProjectedVertex<NumericT>, 3>,
    std::array<internal::ProjectedVertex<NumericT>, 3>> handleOneClipOut(
         const internal::ClipVertex<NumericT>& in1,
         const internal::ClipVertex<NumericT>& in2,
         const internal::ClipVertex<NumericT>& out,
         const sc::Camera<NumericT, sc::VecArray>& cam)
{
    auto in2ToOut = clipToProjectedVertex(findClipCross(in2, out), cam);
    return {
        {
            clipToProjectedVertex(in1, cam),
            clipToProjectedVertex(in2, cam),
            in2ToOut,
        },
        {
            clipToProjectedVertex(in1, cam),
            in2ToOut,
            clipToProjectedVertex(findClipCross(in1, out), cam),
        }
    };
}

template<typename NumericT>
bool triangleTriviallyClipped(
    const internal::ClipVertex<NumericT>& v0,
    const internal::ClipVertex<NumericT>& v1,
    const internal::ClipVertex<NumericT>& v2
) {
    auto outsideLeft   = [](const auto& v) { return v.clip[3] > 0 && v.clip[0] < -v.clip[3]; };
    auto outsideRight  = [](const auto& v) { return v.clip[3] > 0 && v.clip[0] >  v.clip[3]; };
    auto outsideBottom = [](const auto& v) { return v.clip[3] > 0 && v.clip[1] < -v.clip[3]; };
    auto outsideTop    = [](const auto& v) { return v.clip[3] > 0 && v.clip[1] >  v.clip[3]; };
    auto outsideNear   = [](const auto& v) { return v.clip[2] < -v.clip[3]; };
    auto outsideFar    = [](const auto& v) { return v.clip[3] > 0 && v.clip[2] >  v.clip[3]; };

    if (outsideLeft(v0) && outsideLeft(v1) && outsideLeft(v2)) return true;
    if (outsideRight(v0) && outsideRight(v1) && outsideRight(v2)) return true;
    if (outsideBottom(v0) && outsideBottom(v1) && outsideBottom(v2)) return true;
    if (outsideTop(v0) && outsideTop(v1) && outsideTop(v2)) return true;
    if (outsideNear(v0) && outsideNear(v1) && outsideNear(v2)) return true;
    if (outsideFar(v0) && outsideFar(v1) && outsideFar(v2)) return true;

    return false;
}


template<typename NumericT, typename FragmentShader>
void rasterizeTriangle(
    const std::array<internal::ProjectedVertex<NumericT>, 3>& tri,
    const FragmentShader& shader,
    internal::SceneCache<NumericT>& cache)
{
    auto& renderer = cache.renderer;
    auto& zBuffer = cache.zBuffer;

    const auto& v0 = tri[0].pixel;
    const auto& v1 = tri[1].pixel;
    const auto& v2 = tri[2].pixel;

    int tmp = static_cast<int>(std::min({v0[0], v1[0], v2[0]}));
    const int minX = std::max(0, tmp);

    tmp = static_cast<int>(std::max({v0[0], v1[0], v2[0]}));
    const int maxX = std::min(static_cast<int>(renderer.getRenderWidth()) - 1, tmp);

    tmp = static_cast<int>(std::min({v0[1], v1[1], v2[1]}));
    const int minY = std::max(0, tmp);

    tmp = static_cast<int>(std::max({v0[1], v1[1], v2[1]}));
    const int maxY = std::min(static_cast<int>(renderer.getRenderHeight()) - 1, tmp);

    const NumericT area = edgeFunction(v0, v1, v2);
    if (area == 0) return;
    const NumericT invArea = NumericT(1) / area;

    
    const auto a0 = tri[0].attr * tri[0].invW;
    const auto a1 = tri[1].attr * tri[1].invW;
    const auto a2 = tri[2].attr * tri[2].invW;

    for (int y = minY; y <= maxY; ++y)
    {
        for (int x = minX; x <= maxX; ++x)
        {
            sc::utils::Vec<NumericT, 2> p{NumericT(x) + NumericT(0.5),
                                           NumericT(y) + NumericT(0.5)};

            
            NumericT e0 = edgeFunction(v1, v2, p);
            NumericT e1 = edgeFunction(v2, v0, p);
            NumericT e2 = edgeFunction(v0, v1, p);

            if ((area > 0 && e0 >= 0 && e1 >= 0 && e2 >= 0) ||
                (area < 0 && e0 <= 0 && e1 <= 0 && e2 <= 0))
            {
                
                NumericT w0 = e0 * invArea;
                NumericT w1 = e1 * invArea;
                NumericT w2 = e2 * invArea;

                NumericT baryInvW =
                    w0 * tri[0].invW +
                    w1 * tri[1].invW +
                    w2 * tri[2].invW;
                NumericT currentZ = NumericT(1) / baryInvW;

                if (currentZ > 0 && currentZ < zBuffer[y][x])
                {
                    
                    auto interpAttr = (a0 * w0 + a1 * w1 + a2 * w2) * currentZ;

                    FragmentInput<NumericT> frag{
                        interpAttr.uv,
                        interpAttr.normal,
                        interpAttr.worldPos,
                        cache.camera.pos(),
                        currentZ,
                        cache.lights
                    };

                    auto color = shader(frag);

                    zBuffer[y][x] = currentZ;
                    renderer.setPixel(x, y, color);
                }
            }
        }
    }
}

template<typename NumericT, typename FragmentShader>
void processTriangle(
    const std::array<internal::ClipVertex<NumericT>, 3>& clipVerts,
    const FragmentShader& shader,
    internal::SceneCache<NumericT>& cache)
{
    const auto& clip0 = clipVerts[0];
    const auto& clip1 = clipVerts[1];
    const auto& clip2 = clipVerts[2];

    if (triangleTriviallyClipped(clip0, clip1, clip2))
        return;

    const bool clip0Out = clip0.clip[2] < -clip0.clip[3];
    const bool clip1Out = clip1.clip[2] < -clip1.clip[3];
    const bool clip2Out = clip2.clip[2] < -clip2.clip[3];

    int outCount = static_cast<int>(clip0Out) +
                   static_cast<int>(clip1Out) +
                   static_cast<int>(clip2Out);

    auto& camera = cache.camera;

    if (outCount == 1)
    {
        internal::ClipVertex<NumericT> in[2];
        internal::ClipVertex<NumericT> out;

        int inIdx = 0;
        if (!clip0Out) in[inIdx++] = clip0; else out = clip0;
        if (!clip1Out) in[inIdx++] = clip1; else out = clip1;
        if (!clip2Out) in[inIdx]   = clip2; else out = clip2;

        auto [tri1, tri2] = handleOneClipOut(in[0], in[1], out, camera);
        rasterizeTriangle(tri1,shader, cache);
        rasterizeTriangle(tri2, shader, cache);
        return;
    }

    std::array<internal::ProjectedVertex<NumericT>, 3> targetTriangle;
    if (outCount == 2)
    {
        internal::ClipVertex<NumericT> out[2];
        internal::ClipVertex<NumericT> in;

        int inIdx = 0;
        if (clip0Out) out[inIdx++] = clip0; else in = clip0;
        if (clip1Out) out[inIdx++] = clip1; else in = clip1;
        if (clip2Out) out[inIdx]   = clip2; else in = clip2;

        targetTriangle = handleTwoClipOut(out[0], out[1], in, camera);
    }
    else 
    {
        targetTriangle = {
            clipToProjectedVertex(clip0, camera),
            clipToProjectedVertex(clip1, camera),
            clipToProjectedVertex(clip2, camera)
        };
    }

    rasterizeTriangle(targetTriangle, shader, cache);
}

} 
