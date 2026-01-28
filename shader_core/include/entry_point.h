#pragma once

#include "camera/camera.h"
#include "render.h"
#include "glfw_render.h"
#include "camera/camera_view_iterator.h"

#include <chrono>
#include <thread>

#ifdef ENABLE_LOG
#include <iostream>
#endif

namespace sc
{

template<typename T, std::size_t N>
using VecArray = utils::Vec<T, N>;

namespace renderInternal {

template<typename NumericT,
    typename RenderFunction,
    typename EachFrameUpdate = decltype([](std::size_t, std::size_t){ })>
void initRender(
const Camera<NumericT, VecArray>& camera,
    RenderFunction rf,
    EachFrameUpdate efu = { },
    const std::vector<std::pair<int, std::function<void()>>>& keyHandlers = { },
    const std::function<void(double, double)>& mouseHandler = { },
    utils::Vec<int, 2> windowResolution = utils::Vec<int, 2>{-1, -1},
    unsigned int targetFrameRateMs = 60)
{
    using namespace std::chrono_literals;

    const float targetFrameRateMsPerFrame = 1000.f / static_cast<float>(targetFrameRateMs);

    utils::Vec<int, 2> windowRes = windowResolution;
    if (windowRes[0] <= 0 || windowRes[1] <= 0)
    {
        windowRes[0] = camera.res()[0];
        windowRes[1] = camera.res()[1];
    }

    GLFWRenderer renderer(
        camera.res()[0],
        camera.res()[1],
            windowRes
        );

    for (const auto& keyHandler : keyHandlers)
    {
        renderer.onKey(keyHandler.first, keyHandler.second);
    }

    renderer.onMouseMove(mouseHandler);

    std::size_t time = 0;
    std::size_t frame = 0;
    while (!renderer.shouldClose())
    {
        auto start = std::chrono::system_clock::now();
        rf(renderer, frame++, time);
        renderer.show();
        efu(frame, time);
        auto end = std::chrono::system_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        int targetFrameRateDiff = static_cast<int>(targetFrameRateMsPerFrame) - static_cast<int>(elapsedTime);
        if (targetFrameRateDiff > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(targetFrameRateDiff));
        end = std::chrono::system_clock::now();
        elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        time += elapsedTime;
#ifdef ENABLE_LOG
        std::cout << "[FRAME]: " << frame << " ";
        std::cout << "[TIME ms]:  " << time << " ";
        std::cout << "[RENDER TIME ms]: " << elapsedTime << " ";
        std::cout << "[FPS]: " << 1000. / static_cast<float>(elapsedTime) << "\n";
#endif
    }
}

} // namespace renderInternal

template<typename NumericT, typename FrameFunction, typename EachFrameUpdate = decltype([](std::size_t, std::size_t){ })>
void initPerFrameRender(
    const Camera<NumericT, VecArray>& camera,
    FrameFunction ff,
    EachFrameUpdate efu = { },
    const std::vector<std::pair<int, std::function<void()>>>& keyHandlers = { },
    const std::function<void(double, double)>& mouseHandler = { },
    utils::Vec<int, 2> windowResolution = utils::Vec<int, 2>{-1, -1},
    unsigned int targetFrameRateMs = 60)
{
    renderInternal::initRender(camera,
        [&ff](GLFWRenderer& renderer, std::size_t frame, std::size_t time) {
            renderer.clear();
            ff(renderer, frame, time);
        },
        efu,
        keyHandlers,
        mouseHandler,
        windowResolution,
        targetFrameRateMs
    );
}

template<typename NumericT, typename ShadeFunction, typename EachFrameUpdate = decltype([](std::size_t, std::size_t){ })>
void initEachPixelRender(
    const Camera<NumericT, VecArray>& camera,
    ShadeFunction sf,
    EachFrameUpdate efu = { },
    const std::vector<std::pair<int, std::function<void()>>>& keyHandlers = { },
    const std::function<void(double, double)>& mouseHandler = { },
    utils::Vec<int, 2> windowResolution = utils::Vec<int, 2>{-1, -1},
    unsigned int targetFrameRateMs = 60)
{
    renderInternal::initRender(camera,
        [&camera, &sf](GLFWRenderer& renderer, std::size_t frame, const std::size_t time) {
            render(renderer, sc::makeViewFromCamera(camera), frame, time, sf);
        },
        efu,
        keyHandlers,
        mouseHandler,
        windowResolution,
        targetFrameRateMs
    );
}

} // namespace sc