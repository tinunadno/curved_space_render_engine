#include "main_pipeline.h"
#include "model/io.h"

int main() {
    sc::Camera<float, sc::VecArray> camera;
    camera.pos()[2] = 2.0f;
    camera.setLen(0.3);
    camera.setRes(sc::utils::Vec<float, 2>{1000, 800});
    const std::string obj1File = std::string(PROJECT_DIR) + "/objects/rock.obj";

    std::vector<mrc::Model<float>> models;
    std::vector<mrc::LightSource<float>> ls;
    models.emplace_back(mrc::io::readFromObjFile<float>(obj1File.c_str()));
    ls.emplace_back(
        sc::utils::Vec<float, 3>{0.f, 1.3f, 0.f},
        sc::utils::Vec<float, 3>{0.f, -1.f, 0.f},
        sc::utils::Vec<float, 3>{1.f, 1.f, 1.f},
        1.f
    );

    std::vector<std::pair<int, std::function<void()>>> customKeyHandlers = {
        {GLFW_KEY_I, [&ls](){ ls[0].position[2] -= .1f; }},
        {GLFW_KEY_J, [&ls](){ ls[0].position[0] -= .1f; }},
        {GLFW_KEY_K, [&ls](){ ls[0].position[2] += .1f; }},
        {GLFW_KEY_L, [&ls](){ ls[0].position[0] += .1f; }},
        {GLFW_KEY_U, [&ls](){ ls[0].position[1] += .1f; }},
        {GLFW_KEY_O, [&ls](){ ls[0].position[1] -= .1f; }},
    };


    mrc::initMrcRender(camera, models, ls, {}, {}, customKeyHandlers);

    return 0;
}
