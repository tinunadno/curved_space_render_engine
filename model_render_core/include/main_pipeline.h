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
void renderSingleFrame(std::vector<Model<NumericT>>& models,
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

} // namespace internal

template<typename NumericT,
    typename EachFrameModelUpdate = decltype([](std::size_t, std::size_t){ }),
    typename CustomDrawer =
        decltype([](std::size_t, std::size_t, sc::GLFWRenderer&, const sc::utils::Mat<NumericT, 4, 4>&){ })>
void initMrcRender(sc::Camera<NumericT, sc::VecArray>& camera,
                   std::vector<Model<NumericT>>& models,
                   EachFrameModelUpdate efmu = { },
                   CustomDrawer cd = { })
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
        cd(frame, time, renderer, viewProj);
    };
    sc::initPerFrameRender(camera, ff);
}

} // namespace mrc