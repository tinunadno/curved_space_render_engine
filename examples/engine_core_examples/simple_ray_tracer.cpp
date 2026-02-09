#include "entry_point.h"

using namespace sc::utils;

// just solving quadratic equation and returns t parameter for ray parametric equasion
float intersectSphere(const Ray<float, 3>& ray, const Vec<float, 3>& center, float radius)
{
    float t = -1.f;
    auto oc = ray.pos() - center;
    float a = dot(ray.rot(), ray.rot());
    float b = 2.f * dot(oc, ray.rot());
    float c = dot(oc, oc) - radius * radius;
    float d = b * b - 4 * a * c;
    if (d >= 0)
    {
        float sd = std::sqrt(d);
        float t1 = (-b - sd) / (2 * a);
        float t2 = (-b + sd) / (2 * a);
        if (t1 > 1e-4f)
        {
            t = t1;
        } else if (t2 > 1e-4)
        {
            t = t2;
        }
    }
    return t;
}

// shader params
static Vec<float, 3> sphereCenter{0.f, 0.f, 0.f};
static Vec<float, 3> sphereColor{1.f, 0.f, 0.f};
static Vec<float, 3> specularColor{1.f, 1.f, 1.f};
static float shininess = 30.f;

// light params
static Vec<float, 3> lightPos{10.f, -10.f, 10.f};

// sphere params
static float sphereRadius = .5f;
static Vec<float, 3> ambient{.1f, .1f, .1f};

Vec<float, 3> shaderFunction(const sc::PixelSample<float>& ps, std::size_t frame, std::size_t)
{
    const auto& ray = ps.ray;
    float t = intersectSphere(ray, sphereCenter, sphereRadius);
    if (t < 0)
    {
        return ambient;
    }

    Vec<float, 3> hitPos = ray.pos() + ray.rot() * t;
    Vec<float, 3> normal = norm(hitPos - sphereCenter);
    Vec<float, 3> ret{0.f, 0.f, 0.f};

    Vec<float, 3> lightVector = lightPos - hitPos;
    Vec<float, 3> lightDir = norm(lightVector);

    ret += sphereColor * std::max(dot(normal, lightDir), 0.f);
    Vec<float, 3> viewDir = norm(ray.rot() * -1.f);

    Vec<float, 3> halfDir = norm(lightDir + viewDir);

    float shininessCoef = std::max(dot(normal, halfDir), 0.f);
    float specular = std::pow(shininessCoef, shininess);

    ret += specularColor * specular;
    return clamp(ret, 0.f, 1.f);
}

int main()
{

    sc::Camera<float, sc::VecArray> camera;
    camera.pos() = Vec<float, 3>{0.f, 0.f, 2.f};
    camera.rot() = Vec<float, 3>{0.f, 0.f, 0.f};
    // camera.setRes(Vec<float, 2>{51, 51}); // just an example
    Vec<float, 3> rotationVec{.0f, .1f, .0f};
    camera.setLen(0.3);
    sc::initEachPixelRender(camera, shaderFunction,
    [&camera, &rotationVec](std::size_t frame, std::size_t)
        {
            camera.pos() = rotateEuler(camera.pos(), rotationVec);
            camera.rot()[1] += .1f;
            camera.pos()[1] = std::sin(static_cast<float>(frame) / 100.f);
        }
        // , Vec<int, 2>{1000, 1000} // just an example
        );

    return 0;
}