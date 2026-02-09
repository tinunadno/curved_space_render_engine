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
               2. Нормаль (world-space)
               =============================== */
            Vec3f geoN = sc::utils::norm(Vec3f{
                static_cast<float>(frag.normal[0]),
                static_cast<float>(frag.normal[1]),
                static_cast<float>(frag.normal[2])
            });

            Vec3f N = geoN;

            if (mat.normalMap)
            {
                // Normal map хранит нормали в tangent space [0,1] → remap [-1,1]
                Vec3f tsN = mat.normalMap->sample(frag.uv);
                tsN = tsN * 2.f - Vec3f{1.f, 1.f, 1.f};

                // Построение TBN базиса
                Vec3f T = sc::utils::norm(Vec3f{
                    static_cast<float>(frag.tangent[0]),
                    static_cast<float>(frag.tangent[1]),
                    static_cast<float>(frag.tangent[2])
                });

                // Gram-Schmidt: ортогонализируем T относительно N
                T = sc::utils::norm(T - geoN * sc::utils::dot(T, geoN));
                Vec3f B = sc::utils::cross(geoN, T);

                // tangent space → world space
                N = sc::utils::norm(
                    T * tsN[0] + B * tsN[1] + geoN * tsN[2]
                );
            }

            /* ===============================
               3. Вектор на камеру (view)
               =============================== */
            Vec3f V = sc::utils::norm(Vec3f{
                static_cast<float>(frag.cameraPos[0] - frag.worldPos[0]),
                static_cast<float>(frag.cameraPos[1] - frag.worldPos[1]),
                static_cast<float>(frag.cameraPos[2] - frag.worldPos[2])
            });

            /* ===============================
               4. Ambient
               =============================== */
            float amb = static_cast<float>(mat.ambient);
            Vec3f result = albedo * amb;

            /* ===============================
               5. Roughness → Phong параметры
               =============================== */
            float shin;
            float ks;

            if (mat.roughnessMap)
            {
                // map_Ns (Blender) хранит roughness:
                //   0 (чёрный) = гладкий,  1 (белый) = шершавый
                float roughness = std::clamp(
                    mat.roughnessMap->sample(frag.uv)[0], 0.f, 1.f);
                float smoothness = 1.f - roughness;
                float s2 = smoothness * smoothness;
                float s4 = s2 * s2;

                // Roughness → Phong shininess:
                //   smooth (r=0) → shin=512, rough (r=1) → shin=2
                shin = s4 * 510.f + 2.f;

                // Roughness → specular intensity:
                //   smooth → full specular, rough → none
                float specBase = static_cast<float>(mat.specular);
                if (specBase <= 0.f) specBase = 0.5f;
                ks = s2 * specBase;
            }
            else
            {
                shin = static_cast<float>(mat.shininess);
                ks   = static_cast<float>(mat.specular);
            }

            /* ===============================
               6. Освещение от всех источников
               =============================== */
            for (const auto& light : frag.lights)
            {
                /* Направление на источник */
                Vec3f L = sc::utils::norm(Vec3f{
                    static_cast<float>(light.position[0] - frag.worldPos[0]),
                    static_cast<float>(light.position[1] - frag.worldPos[1]),
                    static_cast<float>(light.position[2] - frag.worldPos[2])
                });

                float intensity = static_cast<float>(light.intensity);
                float NdotL = std::max(0.f, sc::utils::dot(N, L));

                /* ===== Диффузная (Lambert) ===== */
                Vec3f diffuse = albedo * light.color * (NdotL * intensity);

                /* ===== Specular (Blinn-Phong) ===== */
                Vec3f specVec{0.f, 0.f, 0.f};
                if (ks > 0.f && NdotL > 0.f)
                {
                    // Blinn-Phong: H = norm(L + V), spec = pow(dot(N,H), shin)
                    // Визуально ближе к GGX чем классический Phong
                    Vec3f H = sc::utils::norm(L + V);
                    float NdotH = std::max(0.f, sc::utils::dot(N, H));
                    float spec = std::pow(NdotH, shin);
                    specVec = light.color * (spec * ks * intensity);
                }

                result += diffuse + specVec;
            }

            /* ===============================
               7. Clamp [0, 1]
               =============================== */
            return Vec3f{
                std::min(result[0], 1.f),
                std::min(result[1], 1.f),
                std::min(result[2], 1.f)
            };
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

            // Compute per-face tangent/bitangent from positions + UVs
            using Vec3 = sc::utils::Vec<NumericT, 3>;
            using Vec2 = sc::utils::Vec<NumericT, 2>;

            auto p0 = transformedVerts[face[0][0]];
            auto p1 = transformedVerts[face[1][0]];
            auto p2 = transformedVerts[face[2][0]];

            Vec2 uv0{}, uv1{}, uv2{};
            if (face[0][1] < model.uv().size()) uv0 = model.uv()[face[0][1]];
            if (face[1][1] < model.uv().size()) uv1 = model.uv()[face[1][1]];
            if (face[2][1] < model.uv().size()) uv2 = model.uv()[face[2][1]];

            auto edge1 = p1 - p0;
            auto edge2 = p2 - p0;
            auto duv1 = uv1 - uv0;
            auto duv2 = uv2 - uv0;

            NumericT det = duv1[0] * duv2[1] - duv2[0] * duv1[1];

            Vec3 faceTangent{1, 0, 0};
            Vec3 faceBitangent{0, 1, 0};

            if (std::abs(det) > NumericT(1e-8))
            {
                NumericT invDet = NumericT(1) / det;
                faceTangent   = sc::utils::norm(
                    (edge1 * duv2[1] - edge2 * duv1[1]) * invDet);
                faceBitangent = sc::utils::norm(
                    (edge2 * duv1[0] - edge1 * duv2[0]) * invDet);
            }

            std::array<internal::ClipVertex<NumericT>, 3> clipVerts;
            for (int i = 0; i < 3; ++i)
            {
                auto wsPos = transformedVerts[face[i][0]];

                internal::VertexAttributes<NumericT> attr;
                attr.worldPos  = wsPos;
                attr.tangent   = faceTangent;
                attr.bitangent = faceBitangent;

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
                   const std::vector<LightSource<NumericT>>& lights,
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
                   const std::vector<LightSource<NumericT>>& lights,
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

    auto ff = [&models, &camera, zBuffer,
               &lights,
               efmu = std::move(efmu), cd = std::move(cd),
               makeShader = std::move(makeShader)](
        sc::GLFWRenderer& renderer, std::size_t frame, std::size_t time) mutable
    {
        for (auto& buf : *zBuffer)
            std::fill(buf.begin(), buf.end(), std::numeric_limits<NumericT>::max());

        internal::SceneCache<NumericT> sceneCache{
            renderer,
            camera,
            *zBuffer,
            lights,
        };

        Mat4 view = getViewMatrix(camera);
        Mat4 proj = getProjectionMatrix(camera);
        auto viewProj = proj * view;

        efmu(frame, time);
        internal::renderSingleFrame(models, viewProj, makeShader, sceneCache);
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
