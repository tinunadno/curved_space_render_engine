#pragma once

#include "camera/camera.h"
#include "render.h"
#include "glfw_render.h"
#include "camera/camera_view_iterator.h"

#include <chrono>
#include <thread>
#include <utility>

#ifdef ENABLE_LOG
#include <iostream>
#endif

namespace sc
{

template<typename T, std::size_t N>
using VecArray = utils::Vec<T, N>;

template<typename RenderFunc, typename FrameUpdate>
class Window
{
public:
    Window(GLFWRenderer&& renderer,
           RenderFunc rf,
           FrameUpdate efu,
           float targetMsPerFrame)
        : _renderer(std::move(renderer))
        , _rf(std::move(rf))
        , _efu(std::move(efu))
        , _targetMsPerFrame(targetMsPerFrame)
    {}

    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;
    Window(Window&&) = default;
    Window& operator=(Window&&) = delete;

    /// Render one frame: call render func, present, call frame update.
    void tick()
    {
        if (shouldClose()) return;

        auto start = std::chrono::system_clock::now();
        _rf(_renderer, _frame, _time);
        _renderer.present();
        _efu(_frame, _time);
        ++_frame;
        auto end = std::chrono::system_clock::now();
        auto elapsedTime = std::chrono::duration_cast<
            std::chrono::milliseconds>(end - start).count();
        _time += elapsedTime;

#ifdef ENABLE_LOG
        std::cout << "[FRAME]: " << _frame << " ";
        std::cout << "[TIME ms]:  " << _time << " ";
        std::cout << "[RENDER TIME ms]: " << elapsedTime << " ";
        std::cout << "[FPS]: " << 1000. / static_cast<float>(elapsedTime) << "\n";
#endif
    }

    [[nodiscard]] bool shouldClose() const { return _renderer.shouldClose(); }

    GLFWRenderer& renderer() { return _renderer; }
    const GLFWRenderer& renderer() const { return _renderer; }

private:
    GLFWRenderer _renderer;
    RenderFunc _rf;
    FrameUpdate _efu;
    float _targetMsPerFrame;
    std::size_t _frame = 0;
    std::size_t _time = 0;
};

namespace windowInternal
{

template<typename NumericT>
GLFWRenderer makeRenderer(
    const Camera<NumericT, VecArray>& camera,
    utils::Vec<int, 2> windowResolution,
    const char* title,
    const std::vector<std::pair<int, std::function<void()>>>& keyHandlers,
    const std::function<void(double, double)>& mouseHandler)
{
    utils::Vec<int, 2> windowRes(windowResolution);
    if (windowRes[0] <= 0 || windowRes[1] <= 0)
    {
        windowRes[0] = static_cast<int>(camera.res()[0]);
        windowRes[1] = static_cast<int>(camera.res()[1]);
    }

    GLFWRenderer renderer(
        static_cast<int>(camera.res()[0]),
        static_cast<int>(camera.res()[1]),
        windowRes,
        title
    );

    for (const auto& [key, handler] : keyHandlers)
        renderer.onKey(key, handler);

    if (mouseHandler)
        renderer.onMouseMove(mouseHandler);

    return renderer;
}

} // namespace windowInternal

template<typename NumericT,
         typename FrameFunc,
         typename EachFrameUpdate = decltype([](std::size_t, std::size_t){ })>
auto makePerFrameWindow(
    const Camera<NumericT, VecArray>& camera,
    FrameFunc ff,
    EachFrameUpdate efu = { },
    const std::vector<std::pair<int, std::function<void()>>>& keyHandlers = { },
    const std::function<void(double, double)>& mouseHandler = { },
    utils::Vec<int, 2> windowResolution = utils::Vec<int, 2>{-1, -1},
    unsigned int targetFrameRateMs = 60,
    const char* title = "Renderer")
{
    const float targetMsPerFrame = 1000.f / static_cast<float>(targetFrameRateMs);

    auto renderer = windowInternal::makeRenderer(
        camera, windowResolution, title, keyHandlers, mouseHandler);

    auto wrapped = [ff = std::move(ff)](
        GLFWRenderer& r, std::size_t frame, std::size_t time) mutable
    {
        r.clear();
        ff(r, frame, time);
    };

    return Window(std::move(renderer), std::move(wrapped), std::move(efu), targetMsPerFrame);
}

template<typename NumericT,
         typename ShadeFunc,
         typename EachFrameUpdate = decltype([](std::size_t, std::size_t){ })>
auto makeEachPixelWindow(
    const Camera<NumericT, VecArray>& camera,
    ShadeFunc sf,
    EachFrameUpdate efu = { },
    const std::vector<std::pair<int, std::function<void()>>>& keyHandlers = { },
    const std::function<void(double, double)>& mouseHandler = { },
    utils::Vec<int, 2> windowResolution = utils::Vec<int, 2>{-1, -1},
    unsigned int targetFrameRateMs = 60,
    const char* title = "Renderer")
{
    const float targetMsPerFrame = 1000.f / static_cast<float>(targetFrameRateMs);

    auto renderer = windowInternal::makeRenderer(
        camera, windowResolution, title, keyHandlers, mouseHandler);

    auto wrapped = [&camera, sf = std::move(sf)](
        GLFWRenderer& r, std::size_t frame, std::size_t time) mutable
    {
        render(r, makeViewFromCamera(camera), frame, time, sf);
    };

    return Window(std::move(renderer), std::move(wrapped), std::move(efu), targetMsPerFrame);
}

template<typename... Windows>
void runWindows(Windows&... windows)
{
    // With >1 window, disable vsync to avoid per-window blocking
    if constexpr (sizeof...(Windows) > 1)
    {
        ([&] {
            windows.renderer().makeContextCurrent();
            glfwSwapInterval(0);
        }(), ...);
    }

    constexpr float targetMsPerFrame = 1000.f / 60.f;

    bool anyOpen = true;
    while (anyOpen)
    {
        auto loopStart = std::chrono::system_clock::now();

        glfwPollEvents();
        anyOpen = false;

        ([&] {
            if (!windows.shouldClose())
            {
                windows.tick();
                anyOpen = true;
            }
        }(), ...);

        // For multi-window: single sleep to target ~60 fps
        if constexpr (sizeof...(Windows) > 1)
        {
            auto loopEnd = std::chrono::system_clock::now();
            auto elapsed = std::chrono::duration_cast<
                std::chrono::milliseconds>(loopEnd - loopStart).count();
            int diff = static_cast<int>(targetMsPerFrame) - static_cast<int>(elapsed);
            if (diff > 0)
                std::this_thread::sleep_for(std::chrono::milliseconds(diff));
        }
    }
}

} // namespace sc
