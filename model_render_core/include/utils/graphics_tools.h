#pragma once
#include "glfw_render.h"

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
            x0 < renderer.getRenderWidth() &&
            y0 < renderer.getRenderHeight())
        {
            renderer.setPixel(x0, y0, color);
        }

        if (x0 == x1 && y0 == y1)
            break;

        const int e2 = err << 1;

        if (e2 > -dy)
        {
            err -= dy;
            x0 += sx;
        }

        if (e2 < dx)
        {
            err += dx;
            y0 += sy;
        }
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
    const int minX = std::max(0,tmp);

    tmp = std::max({v0[0], v1[0], v2[0]});
    const int maxX = std::min(static_cast<int>(renderer.getRenderWidth()) - 1, tmp);

    tmp = std::min({v0[1], v1[1], v2[1]});
    const int minY = std::max(0,tmp);

    tmp = std::max({v0[1], v1[1], v2[1]});
    const int maxY = std::min( static_cast<int>(renderer.getRenderHeight()) - 1, tmp);

    const int area = edgeFunction(v0, v1, v2);
    if (area == 0)
        return;

    for (int y = minY; y <= maxY; ++y)
    {
        for (int x = minX; x <= maxX; ++x)
        {
            sc::utils::Vec<NumericT, 2> p{x, y};

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
void drawTriangleByZBuffer(
    sc::GLFWRenderer& renderer,
    const Model<NumericT>& model,
    const std::vector<sc::utils::Vec<NumericT, 3>>& verticies,
    std::size_t faceIdx,
    const sc::utils::Vec<NumericT, 3>& color,
    const sc::Camera<NumericT, sc::VecArray>& camera,
    std::vector<std::vector<NumericT>>& zBuffer,
    const sc::utils::Mat<NumericT, 4, 4>& viewProj)
{
    const auto poly = model.getPolygon(faceIdx, verticies);

    const auto pv0 = projectVertex(poly[0], viewProj, camera);
    const auto pv1 = projectVertex(poly[1], viewProj, camera);
    const auto pv2 = projectVertex(poly[2], viewProj, camera);

    if (pv0.invW <= 0 || pv1.invW <= 0 || pv2.invW <= 0) {
        return;
    }

    const sc::utils::Vec<NumericT, 2>& v0 = pv0.pixel;
    const sc::utils::Vec<NumericT, 2>& v1 = pv1.pixel;
    const sc::utils::Vec<NumericT, 2>& v2 = pv2.pixel;

    int tmp = std::min({v0[0], v1[0], v2[0]});
    const int minX = std::max(0,tmp);

    tmp = std::max({v0[0], v1[0], v2[0]});
    const int maxX = std::min(static_cast<int>(renderer.getRenderWidth()) - 1, tmp);

    tmp = std::min({v0[1], v1[1], v2[1]});
    const int minY = std::max(0,tmp);

    tmp = std::max({v0[1], v1[1], v2[1]});
    const int maxY = std::min( static_cast<int>(renderer.getRenderHeight()) - 1, tmp);

    const NumericT area = edgeFunction(v0, v1, v2);
    if (area == 0)
        return;
    NumericT invArea = NumericT(1.0) / area;

    for (int y = minY; y <= maxY; ++y)
    {
        for (int x = minX; x <= maxX; ++x)
        {
            sc::utils::Vec<NumericT, 2> p{(NumericT)x + 0.5f, (NumericT)y + 0.5f };
            NumericT w0 = edgeFunction(v1, v2, p) * invArea;
            NumericT w1 = edgeFunction(v2, v0, p) * invArea;
            NumericT w2 = edgeFunction(v0, v1, p) * invArea;
            if ((area > 0 && w0 >= 0 && w1 >= 0 && w2 >= 0) ||
                (area < 0 && w0 <= 0 && w1 <= 0 && w2 <= 0))
            {
                NumericT absw0 = std::abs(w0);
                NumericT absw1 = std::abs(w1);
                NumericT absw2 = std::abs(w2);

                float baryInvW = pv0.invW * absw0 + pv1.invW * absw1 + pv2.invW * absw2;
                float currentZ = 1.0f / baryInvW;

                if (currentZ > 0 && currentZ < zBuffer[y][x])
                {
                    zBuffer[y][x] = currentZ;
                    renderer.setPixel(x, y, color);
                }
            }
        }
    }
}


} // namespace mrc::gt