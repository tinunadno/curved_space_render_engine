#include "entry_point.h"

int main()
{

    sc::Camera<float, sc::VecArray> camera;

    auto sf = [](const sc::PixelSample<float>& ps, std::size_t frame, std::size_t)
    {
        const float radius = static_cast<float>(frame % 100) / 100.f;
        if (ps.uv[0] * ps.uv[0] + ps.uv[1] * ps.uv[1] > radius)
            return sc::utils::Vec<float, 3>{0.f, 0.f, 0.f};
        if (ps.uv[0] < 0 && ps.uv[1] < 0)
            return sc::utils::Vec<float, 3>{1.f, 1.f, 1.f};
        if (ps.uv[0] < 0 && ps.uv[1] > 0)
            return sc::utils::Vec<float, 3>{1.f, 0.f, 0.f};
        if (ps.uv[0] > 0 && ps.uv[1] < 0)
            return sc::utils::Vec<float, 3>{0.f, 1.f, 0.f};
        if (ps.uv[0] > 0 && ps.uv[1] > 0)
            return sc::utils::Vec<float, 3>{0.f, 0.f, 1.f};
        return sc::utils::Vec<float, 3>{1.f, 0.f, 0.f};
    };

    auto ff = [&camera, &sf](sc::GLFWRenderer& renderer, std::size_t frame, std::size_t time)
    {
        // basically doing same thing as in color_check but manually for each frame
        for (const auto& ps : sc::makeViewFromCamera(camera))
        {
            sc::utils::Vec<float, 3> color = sf(ps, frame, time);
            renderer.setPixel(ps.pixelX, ps.pixelY, color);
        }
    };
    sc::initPerFrameRender(camera, ff);

    return 0;
}
