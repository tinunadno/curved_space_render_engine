#pragma once
#include "glfw_render.h"
#include "polygon_distance.h"

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
    const sc::utils::Vec<NumericT, 2>& v0,
    const sc::utils::Vec<NumericT, 2>& v1,
    const sc::utils::Vec<NumericT, 2>& v2,
    const sc::utils::Vec<NumericT, 3>& color,
    const sc::Camera<NumericT, sc::VecArray>& camera,
    std::vector<std::vector<NumericT>>& zBuffer)
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
                sc::PixelSample<NumericT> ps = camera.getPixelSample(w0, w1, w2);
                NumericT distance = getDistanceFromRayToPoly(ps, v0, v1, v2);
                if (zBuffer[y][x] > distance)
                {
                    zBuffer[y][x] = distance;
                    renderer.setPixel(x, y, color);
                }
            }
        }
    }
}


} // namespace mrc::gt