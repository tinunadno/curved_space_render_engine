#include "main_pipeline.h"
#include "model/io.h"

int main() {
    sc::Camera<float, sc::VecArray> camera;
    camera.pos()[2] = 2.0f;
    camera.setLen(0.3);
    const std::string objFile = std::string(PROJECT_DIR) + "/monke.obj";

    std::vector<mrc::Model<float>> models;
    std::vector<mrc::LightSource<float>> ls;
    models.emplace_back(mrc::io::readFromObjFile<float>(objFile.c_str()));
    ls.emplace_back(
        sc::utils::Vec<float, 3>{0.f, 10.f, 0.f},
        sc::utils::Vec<float, 3>{0.f, -1.f, 0.f},
        sc::utils::Vec<float, 3>{1.f, 0.f, 0.f},
        1.f
    );

    mrc::initMrcRender(camera, models, ls);

    return 0;
}
