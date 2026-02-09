#pragma once

#include "entry_point.h"
#include "model/model.h"
#include "utils/point_projection.h"
#include "utils/graphics_tools.h"
#include "utils/compute_normals.h"
#include "utils/vertices_transform.h"

#include <memory>
#include <algorithm>

namespace mrc {

namespace internal
{

template<typename NumericT>
struct DefaultShaderFactory
{
    auto operator()(const Model<NumericT>& model) const
    {
        return [&mat = model.material](const FragmentInput<NumericT>& frag)
            -> sc::utils::Vec<float, 3>
        {
            using Vec3f = sc::utils::Vec<float, 3>;

            /* ===============================
               1. Базовый цвет (albedo)
               =============================== */
            Vec3f albedo{
                static_cast<float>(mat.baseColor[0]),
                static_cast<float>(mat.baseColor[1]),
                static_cast<float>(mat.baseColor[2])
            };

            if (mat.diffuseMap)
                albedo = mat.diffuseMap->sample(frag.uv);

            /* ===============================
               2. Нормаль
               =============================== */
            Vec3f N;

            if (mat.normalMap)
            {
                // normal map в [0,1] -> [-1,1]
                Vec3f nm = mat.normalMap->sample(frag.uv);
                N = sc::utils::norm(Vec3f{
                    nm[0] * 2.f - 1.f,
                    nm[1] * 2.f - 1.f,
                    nm[2] * 2.f - 1.f
                });
            }
            else
            {
                N = sc::utils::norm(Vec3f{
                    static_cast<float>(frag.normal[0]),
                    static_cast<float>(frag.normal[1]),
                    static_cast<float>(frag.normal[2])
                });
            }

            /* ===============================
               3. Вектор на камеру
               =============================== */
            Vec3f V = sc::utils::norm(Vec3f{
                -static_cast<float>(frag.worldPos[0]),
                -static_cast<float>(frag.worldPos[1]),
                -static_cast<float>(frag.worldPos[2])
            });

            /* ===============================
               4. Ambient
               =============================== */
            Vec3f result = albedo * static_cast<float>(mat.ambient);

            /* ===============================
               5. Освещение от всех источников
               =============================== */
            for (const auto& light : frag.lights)
            {
                Vec3f L = sc::utils::norm(Vec3f{
                    static_cast<float>(light.position[0] - frag.worldPos[0]),
                    static_cast<float>(light.position[1] - frag.worldPos[1]),
                    static_cast<float>(light.position[2] - frag.worldPos[2])
                });

                float NdotL = std::max(0.f, sc::utils::dot(N, L));

                /* ===== Диффузная (Lambert) ===== */
                Vec3f diffuse =
                    albedo *
                    NdotL *
                    light.intensity *
                    light.color;

                /* ===== Specular (Phong) ===== */
                Vec3f R = sc::utils::norm(2.f * NdotL * N - L);

                float spec = 0.f;
                if (mat.specular > 0.f && NdotL > 0.f)
                {
                    constexpr float shininess = 32.f;
                    spec = std::pow(
                        std::max(0.f, sc::utils::dot(R, V)),
                        shininess
                    );
                }

                Vec3f specular =
                    light.color *
                    spec *
                    static_cast<float>(mat.specular) *
                    light.intensity;

                result += diffuse + specular;
            }

            return result;
        };

    }
};

template<typename NumericT, typename MakeShader>
void renderSingleFrame(const std::vector<Model<NumericT>>& models,
                       const sc::utils::Mat<NumericT, 4, 4>& projView,
                       MakeShader&& makeShader,
                       SceneCache<NumericT>& sceneCache)
{
    for (const auto& model : models)
    {
        auto shader = makeShader(model);

        auto transformedVerts = internal::transformVerticies(
            model.verticies(), model.pos(), model.rot());
        auto transformedNormals = internal::transformNormals(
            model.normals(), model.rot());

        for (std::size_t f = 0; f < model.faces().size(); ++f)
        {
            const auto& face = model.faces()[f];

            
            auto faceNormal = getFaceNormal(transformedVerts, face);

            std::array<internal::ClipVertex<NumericT>, 3> clipVerts;
            for (int i = 0; i < 3; ++i)
            {
                auto wsPos = transformedVerts[face[i][0]];

                internal::VertexAttributes<NumericT> attr;
                attr.worldPos = wsPos;

                
                if (face[i][1] < model.uv().size())
                    attr.uv = model.uv()[face[i][1]];

                
                if (face[i][2] < transformedNormals.size())
                    attr.normal = transformedNormals[face[i][2]];
                else
                    attr.normal = faceNormal;

                clipVerts[i] = wsToClip(wsPos, projView, attr);
            }

            gt::processTriangle(clipVerts, shader, sceneCache);
        }
    }
}

template<typename NumericT>
void handleCameraMovement(int axis, NumericT distance,
                          sc::Camera<NumericT, sc::VecArray>& camera)
{
    const auto forward = getForward(camera.rot());
    const auto right   = getRight(forward);
    const auto up      = getUp(forward, right);

    sc::utils::Vec<NumericT, 3> d{0., 0., 0.};
    d[axis] = distance;

    camera.pos() += forward * d[2];
    camera.pos() += right   * d[0];
    camera.pos() += up      * d[1];
}

} 



template<typename NumericT,
    typename EachFrameModelUpdate = decltype([](std::size_t, std::size_t){ }),
    typename CustomDrawer =
        decltype([](std::size_t, std::size_t, sc::GLFWRenderer&, const sc::utils::Mat<NumericT, 4, 4>&,
            const std::vector<std::vector<NumericT>>&){ }),
    typename MakeShader = internal::DefaultShaderFactory<NumericT>>
void initMrcRender(sc::Camera<NumericT, sc::VecArray>& camera,
                   const std::vector<Model<NumericT>>& models,
                   const std::vector<LightSource<NumericT>>& lights = {LightSource<NumericT>{}},
                   EachFrameModelUpdate efmu = { },
                   CustomDrawer cd = { },
                   const std::vector<std::pair<int, std::function<void()>>>& customKeyHandlers = {},
                   sc::utils::Vec<int, 2> windowResolution = sc::utils::Vec<int, 2>{-1, -1},
                   unsigned int targetFrameRateMs = 60,
                   MakeShader makeShader = { })
{
    using Mat4 = sc::utils::Mat<NumericT, 4, 4>;

    std::vector zBuffer(
        static_cast<std::size_t>(camera.res()[1]),
        std::vector(
            static_cast<std::size_t>(camera.res()[0]),
          std::numeric_limits<float>::max()
        )
    );

    const auto usc = [&zBuffer, &camera] {
        for (auto& buf : zBuffer)
            std::fill(buf.begin(), buf.end(), std::numeric_limits<float>::max());
        Mat4 view = mrc::getViewMatrix(camera);
        Mat4 proj = mrc::getProjectionMatrix(camera);
        return std::pair{view, proj};
    };

    auto ff = [&efmu, &usc, &cd, &models, &camera, &zBuffer, &lights, makeShader = std::move(makeShader)](
        sc::GLFWRenderer& renderer, std::size_t frame, std::size_t time) mutable
    {
        internal::SceneCache<NumericT> sceneCache{
            renderer,
            camera,
            zBuffer,
            lights,
        };
        auto [view, proj] = usc();
        auto viewProj = proj * view;
        efmu(frame, time);
        internal::renderSingleFrame(models, viewProj, makeShader, sceneCache);
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



template<typename NumericT,
    typename EachFrameModelUpdate = decltype([](std::size_t, std::size_t){ }),
    typename CustomDrawer =
        decltype([](std::size_t, std::size_t, sc::GLFWRenderer&, const sc::utils::Mat<NumericT, 4, 4>&,
            const std::vector<std::vector<NumericT>>&){ }),
    typename MakeShader = internal::DefaultShaderFactory<NumericT>>
auto makeMrcWindow(sc::Camera<NumericT, sc::VecArray>& camera,
                   const std::vector<Model<NumericT>>& models,
                   const std::vector<LightSource<NumericT>>& lights = {LightSource<NumericT>{}},
                   EachFrameModelUpdate efmu = { },
                   CustomDrawer cd = { },
                   const std::vector<std::pair<int, std::function<void()>>>& customKeyHandlers = {},
                   sc::utils::Vec<int, 2> windowResolution = sc::utils::Vec<int, 2>{-1, -1},
                   unsigned int targetFrameRateMs = 60,
                   const char* title = "Model Renderer",
                   MakeShader makeShader = { })
{
    using Mat4 = sc::utils::Mat<NumericT, 4, 4>;

    auto zBuffer = std::make_shared<std::vector<std::vector<NumericT>>>(
        static_cast<std::size_t>(camera.res()[1]),
        std::vector<NumericT>(
            static_cast<std::size_t>(camera.res()[0]),
            std::numeric_limits<NumericT>::max()
        )
    );

    auto ff = [&models, &camera, zBuffer, &lights,
               efmu = std::move(efmu), cd = std::move(cd),
               makeShader = std::move(makeShader)](
        sc::GLFWRenderer& renderer, std::size_t frame, std::size_t time) mutable
    {
        for (auto& buf : *zBuffer)
            std::fill(buf.begin(), buf.end(), std::numeric_limits<NumericT>::max());

        Mat4 view = getViewMatrix(camera);
        Mat4 proj = getProjectionMatrix(camera);
        auto viewProj = proj * view;

        efmu(frame, time);
        internal::renderSingleFrame(models, lights, camera, *zBuffer, renderer, viewProj, makeShader);
        cd(frame, time, renderer, viewProj, *zBuffer);
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

    return sc::makePerFrameWindow(camera, std::move(ff),
        [](std::size_t, std::size_t){},
        keyHandlers, mouseHandler, windowResolution, targetFrameRateMs, title);
}

} 
