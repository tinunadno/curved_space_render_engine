#include "entry_point.h"
#include "ray_marching_iterator.h"
#include "curvature/spherical_curvature.h"
#include "object/sphere.h"

using fpT = float;

sc::utils::Vec<float, 3> shaderFunction(const sc::PixelSample<fpT>& ps, const rmc::Scene<fpT>& scene)
{
    rmc::RayMarchingResult<fpT> rayMarchingResult = rmc::marchRay(ps.ray, scene);
    if (rayMarchingResult.reachedThreshold)
    {
        return rayMarchingResult.material.color;
    }
    return sc::utils::Vec<float, 3>{.3f, .05f, .2f};
}

int main()
{

    sc::Camera<fpT, sc::VecArray> camera;

    camera.setRes(sc::utils::Vec<fpT, 2>{200, 200});
    camera.pos() = sc::utils::Vec<fpT, 3>{0.f, 0.f, 2.f};
    camera.rot() = sc::utils::Vec<fpT, 3>{0.f, 0.f, 0.f};
    camera.setLen(0.4);

    sc::utils::Vec<fpT, 3> rotationVec{.0f, .1f, .0f};

    auto sphereShape =
    std::make_shared<rmc::object::Sphere<fpT>>(
        sc::utils::Vec<fpT,3>{0.f,0.f,0.f}, 0.5f
    );

    auto sphere1Shape =
    std::make_shared<rmc::object::Sphere<fpT>>(
        sc::utils::Vec<fpT,3>{0.f,0.f,0.f}, 1.f, sc::utils::Vec<fpT,3>{1.f,.2f,1.f}
    );

    rmc::object::SceneObject<fpT> sphere(
        sphereShape,
        sc::utils::Vec<float, 3>{0.f, 0.f, 0.f}
    );

    rmc::object::SceneObject<fpT> sphere1(
        sphere1Shape
    );

    auto bhCurvature = std::make_shared<rmc::curvature::BlackHole<fpT>>();
    bhCurvature->center = sc::utils::Vec<fpT, 3>{0.f, 0.f, 0.f};
    bhCurvature->schwarzschildRadius = .7f;
    bhCurvature->influenceRadius = .7f;
    bhCurvature->strength = .5f;

    rmc::Scene<fpT> scene;
    scene.addObject(sphere);
    scene.addObject(sphere1);
    scene.addCurvature(bhCurvature);

    sc::initEachPixelRender(camera,
        [&scene](const sc::PixelSample<fpT>& ps, std::size_t, std::size_t)
            {
                return shaderFunction(ps, scene);
            },
            [&camera, &rotationVec](std::size_t frame, std::size_t)
        {
            // camera.pos() = rotateEuler(camera.pos(), rotationVec);
            // camera.rot()[1] += .1f;
            camera.pos()[1] = std::sin(static_cast<fpT>(frame) / 100.f);
        }, {}, {},
        sc::utils::Vec<int, 2>{500, 500});
    return 0;
}