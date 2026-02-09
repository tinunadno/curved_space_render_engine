#pragma once

#include "camera/camera_view.h"
#include "utils/vec.h"
#include "glfw_render.h"

namespace sc
{

template<typename NumericT, typename ShadeFunction>
inline void render(
    GLFWRenderer& renderer,
    const internal::CameraView<NumericT>& cameraView,
    std::size_t frame,
    std::size_t timeMs,
    ShadeFunction sf
)
{
    for (const auto& ps : cameraView)
    {
        utils::Vec<float, 3> color = sf(ps, frame, timeMs);
        renderer.setPixel(ps.pixelX, ps.pixelY, color);
    }
}

} // namespace sc