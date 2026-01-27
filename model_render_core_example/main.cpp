#include "entry_point.h"
#include "model.h"
#include "point_projection.h"
#include "utils/graphics_tools.h"

int main() {

    sc::Camera<float, sc::VecArray> camera;
    camera.setLen(0.3);
    const char* objFile = "/Users/yura/stuff/clion/curved_space_render_engine/model_render_core_example/test.obj";

    mrc::Model<float> model = mrc::readFromObjFile<float>(objFile);

    std::vector<sc::utils::Vec<float, 2>> projBuffer(model.verticies().size());



    // eg update projected points
    const auto upp = [&camera, &model, &projBuffer]() {
        const auto projMatrix = mrc::makeProjectionMatrixFromCamera(camera);
        for (std::size_t v = 0; v < model.verticies().size(); ++v) {
            projBuffer[v] = mrc::wsToPixel(model.verticies()[v], projMatrix, camera);
        }
    };

    auto ff = [&projBuffer, &upp, &model](sc::GLFWRenderer& renderer, std::size_t, std::size_t)
    {
        upp();
        for (const auto& v : projBuffer) {
            renderer.setPixel(v[0], v[1], sc::utils::Vec<float, 3>(1., 1., 1.));
        }
        for (const auto& f : model.faces()) {
            mrc::gt::drawLine(
                renderer, projBuffer[f[0][0]], projBuffer[f[1][0]],
                sc::utils::Vec<float, 3>(1., 0., 0.));
            mrc::gt::drawLine(
                renderer, projBuffer[f[1][0]], projBuffer[f[2][0]],
                sc::utils::Vec<float, 3>(1., 0., 0.));
            mrc::gt::drawLine(
                renderer, projBuffer[f[2][0]], projBuffer[f[0][0]],
                sc::utils::Vec<float, 3>(1., 0., 0.));
        }
    };

    sc::utils::Vec<float, 3> rotationVec{.0f, .1f, .0f};
    auto efu = [&camera, &rotationVec](std::size_t frame, std::size_t)
    {
        camera.pos() = rotateEuler(camera.pos(), rotationVec);
        camera.rot()[1] += .1f;
        camera.pos()[1] = std::sin(static_cast<float>(frame) / 100.f);
        std::cout << "cPos " << camera.pos()[0] << " " << camera.pos()[1] << " " << camera.pos()[2] << "\n";
    };
    sc::initPerFrameRender(camera, ff, efu);

    return 0;
}