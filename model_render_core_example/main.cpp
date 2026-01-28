#include "main_pipeline.h"

inline float dotThree(const sc::utils::Vec<float, 2>& a, const sc::utils::Vec<float, 2>& b,
                      const sc::utils::Vec<float, 2>& c) {
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
}

int main() {
    sc::Camera<float, sc::VecArray> camera;
    camera.pos()[2] = 2.0f;
    camera.setLen(0.3);
    const char* objFile = "/Users/yura/stuff/clion/curved_space_render_engine/model_render_core_example/test.obj";

    std::vector<mrc::Model<float>> models;
    models.emplace_back(mrc::readFromObjFile<float>(objFile));

    auto modelUpd = [&models](std::size_t frame, std::size_t) {
        models[0].pos()[1] = std::sin(static_cast<float>(frame) / 100.f);
    };

    auto roiDrawer = [&models, &camera](
        std::size_t,
        std::size_t,
        sc::GLFWRenderer& renderer,
        sc::utils::Mat<float, 4, 4>& proj)
    {
        std::vector<sc::utils::Vec<float, 2>> projectedPoints(models[0].verticies().size());
        for (std::size_t i = 0; i < models[0].verticies().size(); ++i) {
            auto tempV = sc::utils::rotateEuler(models[0].verticies()[i], models[0].rot()) + models[0].pos();
            auto projV = mrc::projectVertex(tempV, proj, camera);
            projectedPoints[i] = projV.pixel;
        }
        std::sort(projectedPoints.begin(), projectedPoints.end(), [](const auto& a, const auto& b) {
            return (a[0] < b[0]) || (a[0] == b[0] && a[1] < b[1]);
        });
        std::vector<std::size_t> upperConvexHullIndices{0, 1};
        for (std::size_t i = 2; i < projectedPoints.size(); ++i) {
            while (upperConvexHullIndices.size() >= 2) {
                const auto& p1 = projectedPoints[*(upperConvexHullIndices.end() - 2)];
                const auto& p2 = projectedPoints[upperConvexHullIndices.back()];
                const auto& p3 = projectedPoints[i];
                if (dotThree(p1, p2, p3) < 0.f) {
                    break;
                }
                upperConvexHullIndices.pop_back();
            }
            upperConvexHullIndices.push_back(i);
        }
        std::vector<std::size_t> hull;
        for (std::size_t i = projectedPoints.size(); i-- > 0; ) {
            while (hull.size() >= 2) {
                const auto& p1 = projectedPoints[*(hull.end() - 2)];
                const auto& p2 = projectedPoints[hull.back()];
                const auto& p3 = projectedPoints[i];
                if (dotThree(p1, p2, p3) < 0.f) {
                    break;
                }
                hull.pop_back();
            }
            hull.push_back(i);
        }

        hull.insert(hull.end(), upperConvexHullIndices.begin() + 1, upperConvexHullIndices.end());

        for (std::size_t i = 1; i < hull.size(); ++i) {
            mrc::gt::drawLine(
                renderer,
                projectedPoints[hull[i]],
                projectedPoints[hull[(i + 1) % (hull.size() - 1)]],
                sc::utils::Vec<float, 3>(1., 0., 0.)
                );
        }
    };

    mrc::initMrcRender(camera, models, modelUpd, roiDrawer);

    return 0;
}
