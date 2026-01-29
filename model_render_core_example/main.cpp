#include "main_pipeline.h"

int main() {
    sc::Camera<float, sc::VecArray> camera;
    camera.pos()[2] = 2.0f;
    camera.setLen(0.3);
    const char* objFile = "/Users/yura/stuff/clion/curved_space_render_engine/model_render_core_example/hy_poly_fract.obj";

    std::vector<mrc::Model<float>> models;
    models.emplace_back(mrc::readFromObjFile<float>(objFile));

    mrc::initMrcRender(camera, models);

    return 0;
}
