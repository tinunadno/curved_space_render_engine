#include "entry_point.h"

// just to check, if i've calculated uv correctly

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
    sc::initEachPixelRender(camera, sf);

    return 0;
}
