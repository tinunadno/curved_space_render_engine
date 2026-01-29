#pragma once

#include "entry_point.h"
#include "model.h"
#include "point_projection.h"
#include "utils/graphics_tools.h"
#include "compute_normals.h"
#include "utils/vertices_transform.h"

namespace mrc {

namespace internal
{

template<typename NumericT>
void renderSingleFrame(const std::vector<Model<NumericT>>& models,
                       const sc::Camera<NumericT, sc::VecArray>& camera,
                       std::vector<std::vector<NumericT>>& zBuffer,
                       sc::GLFWRenderer& renderer,
                       const sc::utils::Mat<NumericT, 4, 4>& projView)
{
    for (const auto& model : models) {
        std::vector<sc::utils::Vec<NumericT, 3>> transformedVerticies =
            internal::transformVerticies(model.verticies(), model.pos(), model.rot());

        for (std::size_t f = 0; f < model.faces().size(); ++f) {
            auto normal = mrc::getFaceNormal(model, f);
            float col = sc::utils::getCos(normal, sc::utils::Vec<float, 3>{1., 0, 0});
            col = (col + 1.f) / 2.f;
            col = col == 0.f ? .1f : col;
            sc::utils::Vec<float, 3> resultingColor{col, col, col};

            gt::drawTriangleByZBuffer(
                renderer,
                model,
                transformedVerticies, f,
                resultingColor,
                camera,
                zBuffer,
                projView);
        }
    }
}

template<typename NumericT>
void handleCameraMovement(int axis, NumericT distance, sc::Camera<NumericT, sc::VecArray>& camera)
{

    const auto forward = getForward(camera.rot());
    const auto right = getRight(forward);
    const auto up = getUp(forward, right);

    sc::utils::Vec<NumericT, 3> d{0., 0., 0.};
    d[axis] = distance;

    camera.pos() += forward * d[2];
    camera.pos() += right * d[0];
    camera.pos() += up * d[1];
}

} // namespace internal

template<typename NumericT,
    typename EachFrameModelUpdate = decltype([](std::size_t, std::size_t){ }),
    typename CustomDrawer =
        decltype([](std::size_t, std::size_t, sc::GLFWRenderer&, const sc::utils::Mat<NumericT, 4, 4>&,
            const std::vector<std::vector<NumericT>>&){ })>
void initMrcRender(sc::Camera<NumericT, sc::VecArray>& camera,
                   const std::vector<Model<NumericT>>& models,
                   EachFrameModelUpdate efmu = { },
                   CustomDrawer cd = { },
                   const std::vector<std::pair<int, std::function<void()>>>& customKeyHandlers = {},
                   sc::utils::Vec<int, 2> windowResolution = sc::utils::Vec<int, 2>{-1, -1},
                   unsigned int targetFrameRateMs = 60)
{
    using Mat4 = sc::utils::Mat<NumericT, 4, 4>;

    std::vector zBuffer(
        static_cast<std::size_t>(camera.res()[1]),
        std::vector(
            static_cast<std::size_t>(camera.res()[0]),
          std::numeric_limits<float>::max()
        )
    );

    // eg update scene cache
    const auto usc = [&zBuffer, &camera] {
        for (auto& buf : zBuffer) {
            std::fill(buf.begin(), buf.end(), std::numeric_limits<float>::max());
        }
        Mat4 view = mrc::getViewMatrix(camera);
        Mat4 proj = mrc::getProjectionMatrix(camera);
        return std::pair{view, proj};
    };

    auto ff = [&efmu, &usc, &cd, &models, &camera, &zBuffer](
        sc::GLFWRenderer& renderer, std::size_t frame, std::size_t time)
    {
        auto [view, proj] = usc();
        auto viewProj = proj * view;
        efmu(frame, time);
        internal::renderSingleFrame(models, camera, zBuffer, renderer, viewProj);
        cd(frame, time, renderer, viewProj, zBuffer);
    };

    constexpr NumericT stepSize = .5;
    constexpr NumericT rotSize = 1. / 100.;

    std::vector<std::pair<int, std::function<void()>>> keyHandlers = {
        {GLFW_KEY_W, [&camera](){ internal::handleCameraMovement(2, NumericT(stepSize), camera); }},
        {GLFW_KEY_A, [&camera](){ internal::handleCameraMovement(0, NumericT(-stepSize), camera); }},
        {GLFW_KEY_S, [&camera](){ internal::handleCameraMovement(2, NumericT(-stepSize), camera); }},
        {GLFW_KEY_D, [&camera](){ internal::handleCameraMovement(0, NumericT(stepSize), camera); }},
        {GLFW_KEY_LEFT_SHIFT, [&camera](){ internal::handleCameraMovement(1, NumericT(stepSize), camera); }},
        {GLFW_KEY_LEFT_CONTROL, [&camera](){ internal::handleCameraMovement(1, NumericT(-stepSize), camera); }},
    };

    keyHandlers.insert(keyHandlers.end(), customKeyHandlers.begin(), customKeyHandlers.end());

    auto mouseHandler = [&camera](double dx, double dy) {
      camera.rot()[0] -= dy * rotSize;
      camera.rot()[1] += dx * rotSize;
    };

    sc::initPerFrameRender(camera, ff, {}, keyHandlers, mouseHandler, windowResolution, targetFrameRateMs);
}

} // namespace mrc