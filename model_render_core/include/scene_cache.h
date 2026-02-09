#pragma once

#include "light_source.h"

#include "glfw_render.h"
#include "window.h"
#include "camera/camera.h"


namespace mrc::internal
{

template<typename NumericT>
struct SceneCache
{
    sc::GLFWRenderer& renderer;
    const sc::Camera<NumericT, sc::VecArray>& camera;
    std::vector<std::vector<NumericT>>& zBuffer;
    const std::vector<LightSource<NumericT>>& lights;
};

} // namespace mrc::internal