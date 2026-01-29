#include "main_pipeline.h"
#include "snapshot.h"
#include "model_intersection.h"

inline float dotThree(const sc::utils::Vec<float, 2>& a, const sc::utils::Vec<float, 2>& b,
                      const sc::utils::Vec<float, 2>& c) {
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
}

inline std::vector<std::size_t> calculateConvexHull(
    std::vector<sc::utils::Vec<float, 2>>& projectedPoints)
{
    std::ranges::sort(projectedPoints, [](const auto& a, const auto& b) {
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

    return hull;

}

std::vector<std::vector<Snapshot>> runFirstRender()
{
    sc::Camera<float, sc::VecArray> camera;
    camera.pos()[2] = 2.0f;
    camera.setLen(0.3);
    const char* objFile = "/Users/yura/stuff/clion/curved_space_render_engine/model_render_core_example/monke.obj";
    const char* obj1File = "/Users/yura/stuff/clion/curved_space_render_engine/model_render_core_example/cube.obj";

    std::vector<mrc::Model<float>> models;
    models.emplace_back(mrc::readFromObjFile<float>(objFile));
    models.emplace_back(mrc::readFromObjFile<float>(obj1File,
        sc::utils::Vec<float, 3>(2., 0., 0.),
        sc::utils::Vec<float, 3>(0., 1.6, 0.)));

    std::vector<std::vector<sc::utils::Vec<float, 2>>> lastRois(models.size());
    std::mutex lastRoiMutex;

    auto roiDrawer = [&models, &camera, &lastRois, &lastRoiMutex](
        std::size_t,
        std::size_t,
        sc::GLFWRenderer& renderer,
        const sc::utils::Mat<float, 4, 4>& proj,
        const std::vector<std::vector<float>>& zBuffer)
    {
        for (std::size_t i = 0; i < models.size(); ++i) {
            const auto& model = models[i];
            std::vector<sc::utils::Vec<float, 2>> projectedPoints;
            for (const auto& v : model.verticies()) {
                auto tempV = sc::utils::rotateEuler(v, model.rot()) + model.pos();
                auto projV = mrc::projectVertex(tempV, proj, camera);
                float currentZ = 1.f / projV.invW;
                if (projV.pixel[0] < 0 || projV.pixel[0] >= zBuffer[0].size()
                    || projV.pixel[1] < 0 || projV.pixel[1] >= zBuffer.size()) {
                    continue;
                }
                if (currentZ > 0 && currentZ - .05f <
                    zBuffer[static_cast<std::size_t>(projV.pixel[1])]
                           [static_cast<std::size_t>(projV.pixel[0])])
                    projectedPoints.emplace_back(projV.pixel);
            }
            auto hull = calculateConvexHull(projectedPoints);
            {
                std::lock_guard lock(lastRoiMutex);
                lastRois[i].clear();
                lastRois[i].reserve(hull.size());

                std::ranges::transform(hull, std::back_inserter(lastRois[i]),
                                       [&projectedPoints](std::size_t i) {
                                           return projectedPoints[i];
                                       });
            }

            for (std::size_t j = 1; j < hull.size(); ++j) {
                mrc::gt::drawLine(
                    renderer,
                    projectedPoints[hull[j]],
                    projectedPoints[hull[(j + 1) % (hull.size() - 1)]],
                    sc::utils::Vec<float, 3>(1., 0., 0.)
                    );
            }
        }
    };

    std::vector<std::vector<Snapshot>> snapshots(models.size());

    std::vector<std::pair<int, std::function<void()>>> customKeyHandlers = { // creating snapshot
        {GLFW_KEY_E, [&snapshots, &camera, &lastRois, &lastRoiMutex]() {
            std::lock_guard lock(lastRoiMutex);
            for (std::size_t i = 0; i < snapshots.size(); ++i) {
                snapshots[i].emplace_back(camera, std::move(lastRois[i]));
            }
        }}
    };

    mrc::initMrcRender(camera, models, { }, roiDrawer, customKeyHandlers);

    return snapshots;
}

std::vector<std::vector<mrc::Model<float>>> runSecondRender(const std::vector<std::vector<Snapshot>>& snapshots)
{
    sc::Camera<float, sc::VecArray> camera;
    camera.pos()[2] = 2.0f;
    camera.setLen(0.3);

    std::vector<std::vector<mrc::Model<float>>> mergedModels;
    std::vector<mrc::Model<float>> models;

    mergedModels.reserve(snapshots.size());

    for (const auto& snapshotGroup : snapshots) {
        auto converted = snapshotsToModels(snapshotGroup);
        mergedModels.emplace_back(converted);
        models.insert(models.end(), converted.begin(), converted.end());
    }

    mrc::initMrcRender(camera, models);
    return mergedModels;
}

void runThirdRender(const std::vector<std::vector<mrc::Model<float>>>& coneModels)
{
    sc::Camera<float, sc::VecArray> camera;
    camera.pos()[2] = 2.0f;
    camera.setLen(0.3);
    std::vector<mrc::Model<float>> models;
    models.reserve(coneModels.size());
    for (const auto& coneModel : coneModels) {
        models.emplace_back(mi::ComputeIntersection(coneModel));
    }
    std::vector objectCenters(models.size(), sc::utils::Vec<float, 3>{0., 0., 0.});

    for (std::size_t m = 0; m < models.size(); ++m) {
        if (models[m].verticies().empty()) continue;
        sc::utils::Vec<float, 3> minV = models[m].verticies()[0];
        sc::utils::Vec<float, 3> maxV = models[m].verticies()[0];

        for (const auto& v : models[m].verticies()) {
            for (int i = 0; i < 3; ++i) {
                if (v[i] < minV[i]) minV[i] = v[i];
                if (v[i] > maxV[i]) maxV[i] = v[i];
            }
        }
        sc::utils::Vec<float, 3> localCenter = (minV + maxV) * 0.5f;
        objectCenters[m] = sc::utils::rotateEuler(localCenter, models[m].rot()) + models[m].pos();
    }

    auto centerDrawer = [&objectCenters, &camera](
        std::size_t,
        std::size_t,
        sc::GLFWRenderer& renderer,
        const sc::utils::Mat<float, 4, 4>& proj,
        const std::vector<std::vector<float>>&) {
        std::vector<sc::utils::Vec<float, 2>> projectedPoints;

        for (const auto& v : objectCenters) {
            auto projV = mrc::projectVertex(v, proj, camera);
            projectedPoints.emplace_back(projV.pixel);
        }

        for (std::size_t i = 0; i < projectedPoints.size(); ++i) {
            for (std::size_t j = i + 1; j < projectedPoints.size(); ++j) {
                mrc::gt::drawLine(renderer,
                    projectedPoints[i],
                    projectedPoints[j],
                    sc::utils::Vec<float, 3>(1., 0., 0.)
                );
            }
        }

    };

    mrc::initMrcRender(camera, models, {}, centerDrawer);
}

int main()
{
    auto snapshots = runFirstRender();
    auto models = runSecondRender(snapshots);
    runThirdRender(models);

    return 0;
}
