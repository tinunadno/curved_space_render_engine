#include "main_pipeline.h"
#include "model/io.h"
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

int main()
{
    sc::Camera<float, sc::VecArray> camera1;
    camera1.pos()[2] = 2.0f;
    camera1.setLen(0.3);

    sc::Camera<float, sc::VecArray> camera2;
    camera2.pos()[2] = 2.0f;
    camera2.setLen(0.3);

    sc::Camera<float, sc::VecArray> camera3;
    camera3.pos()[2] = 2.0f;
    camera3.setLen(0.3);

    const char* objFile = "/Users/yura/stuff/clion/curved_space_render_engine/model_render_core_example/monke.obj";
    const char* obj1File = "/Users/yura/stuff/clion/curved_space_render_engine/model_render_core_example/cube.obj";

    std::vector<mrc::Model<float>> sourceModels;
    std::vector<mrc::LightSource<float>> lights;
    sourceModels.emplace_back(mrc::readFromObjFile<float>(objFile));
    sourceModels.emplace_back(mrc::readFromObjFile<float>(obj1File,
        sc::utils::Vec<float, 3>(2., 0., 0.),
        sc::utils::Vec<float, 3>(0., 1.6, 0.)));

    lights.emplace_back(
        sc::utils::Vec<float, 3>{0.f, 10.f, 0.f},
        sc::utils::Vec<float, 3>{0.f, -1.f, 0.f},
        sc::utils::Vec<float, 3>{1.f, 1.f, 1.f},
        1.f
    );

    lights.emplace_back(
        sc::utils::Vec<float, 3>{10.f, 10.f, 0.f},
        sc::utils::norm(sc::utils::Vec<float, 3>{-10.f, 10.f, 0.f}),
        sc::utils::Vec<float, 3>{1.f, 1.f, 1.f},
        1.f
    );

    std::vector<std::vector<Snapshot>> snapshots(sourceModels.size());
    std::vector<std::vector<sc::utils::Vec<float, 2>>> lastRois(sourceModels.size());

    // cone models (window 2 owns, window 3 reads)
    std::vector<mrc::Model<float>> coneModels;
    std::vector<std::vector<mrc::Model<float>>> groupedCones(sourceModels.size());

    // intersection models (window 3 owns)
    std::vector<mrc::Model<float>> intersectionModels;

    // dirty flags (single-threaded, no mutex needed)
    bool snapshotsDirty = false;
    bool conesDirty = false;

    auto roiDrawer = [&sourceModels, &camera1, &lastRois](
        std::size_t,
        std::size_t,
        sc::GLFWRenderer& renderer,
        const sc::utils::Mat<float, 4, 4>& proj,
        const std::vector<std::vector<float>>& zBuffer)
    {
        for (std::size_t i = 0; i < sourceModels.size(); ++i)
        {
            const auto& model = sourceModels[i];
            std::vector<sc::utils::Vec<float, 2>> projectedPoints;
            for (const auto& v : model.verticies())
            {
                auto tempV = sc::utils::rotateEuler(v, model.rot()) + model.pos();
                auto projV = mrc::projectVertex(tempV, proj, camera1);
                float currentZ = 1.f / projV.invW;
                if (projV.pixel[0] < 0 || projV.pixel[0] >= zBuffer[0].size()
                    || projV.pixel[1] < 0 || projV.pixel[1] >= zBuffer.size())
                    continue;
                if (currentZ > 0 && currentZ - .05f <
                    zBuffer[static_cast<std::size_t>(projV.pixel[1])]
                           [static_cast<std::size_t>(projV.pixel[0])])
                    projectedPoints.emplace_back(projV.pixel);
            }
            auto hull = calculateConvexHull(projectedPoints);

            lastRois[i].clear();
            lastRois[i].reserve(hull.size());
            std::ranges::transform(hull, std::back_inserter(lastRois[i]),
                                   [&projectedPoints](std::size_t idx) {
                                       return projectedPoints[idx];
                                   });

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

    std::vector<std::pair<int, std::function<void()>>> keys1 = {
        {GLFW_KEY_E, [&snapshots, &camera1, &lastRois, &snapshotsDirty]() {
            for (std::size_t i = 0; i < snapshots.size(); ++i)
            {
                std::vector<sc::utils::Vec<float, 2>> roiCopy(lastRois[i]);
                snapshots[i].emplace_back(camera1, std::move(roiCopy));
            }
            snapshotsDirty = true;
        }}
    };

    auto window1 = mrc::makeMrcWindow(camera1, sourceModels, lights,
        [](std::size_t, std::size_t){}, roiDrawer,
        keys1, {}, 60, "Model Viewer");

    auto coneUpdate = [&](std::size_t, std::size_t) {
        if (!snapshotsDirty) return;

        coneModels.clear();
        for (std::size_t i = 0; i < snapshots.size(); ++i)
        {
            auto converted = snapshotsToModels(snapshots[i]);
            groupedCones[i] = converted;
            coneModels.insert(coneModels.end(), converted.begin(), converted.end());
        }
        snapshotsDirty = false;
        conesDirty = true;
    };

    auto window2 = mrc::makeMrcWindow(camera2, coneModels, lights,
        coneUpdate,
        [](std::size_t, std::size_t, sc::GLFWRenderer&,
           const sc::utils::Mat<float, 4, 4>&,
           const std::vector<std::vector<float>>&){},
        {}, {}, 60, "Cone Models");

    auto intersectionUpdate = [&](std::size_t, std::size_t) {
        if (!conesDirty) return;

        intersectionModels.clear();
        for (const auto& group : groupedCones)
        {
            if (!group.empty())
                intersectionModels.emplace_back(mi::ComputeIntersection(group));
        }
        conesDirty = false;
    };

    auto centerDrawer = [&intersectionModels, &camera3](
        std::size_t,
        std::size_t,
        sc::GLFWRenderer& renderer,
        const sc::utils::Mat<float, 4, 4>& viewProj,
        const std::vector<std::vector<float>>&)
    {
        std::vector<sc::utils::Vec<float, 3>> objectCenters;

        for (const auto& model : intersectionModels)
        {
            if (model.verticies().empty()) continue;

            sc::utils::Vec<float, 3> minV = model.verticies()[0];
            sc::utils::Vec<float, 3> maxV = model.verticies()[0];
            for (const auto& v : model.verticies())
            {
                for (int i = 0; i < 3; ++i)
                {
                    if (v[i] < minV[i]) minV[i] = v[i];
                    if (v[i] > maxV[i]) maxV[i] = v[i];
                }
            }
            auto localCenter = (minV + maxV) * 0.5f;
            objectCenters.push_back(
                sc::utils::rotateEuler(localCenter, model.rot()) + model.pos());
        }

        for (std::size_t i = 0; i < objectCenters.size(); ++i)
        {
            auto projI = mrc::projectVertex(objectCenters[i], viewProj, camera3);
            for (std::size_t j = i + 1; j < objectCenters.size(); ++j)
            {
                auto projJ = mrc::projectVertex(objectCenters[j], viewProj, camera3);
                mrc::gt::drawLine(renderer,
                    projI.pixel, projJ.pixel,
                    sc::utils::Vec<float, 3>(1., 0., 0.));
            }
        }
    };

    auto window3 = mrc::makeMrcWindow(camera3, intersectionModels, lights,
        intersectionUpdate, centerDrawer,
        {}, {}, 60, "Intersections");

    sc::runWindows(window1, window2, window3);

    return 0;
}
