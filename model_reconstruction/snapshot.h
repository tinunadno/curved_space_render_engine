#pragma once

#include "entry_point.h"
#include "utils/vec.h"
#include "camera/camera.h"
#include "point_projection.h"

struct Snapshot
{
    sc::utils::Vec<float, 3> cameraPosition;
    sc::utils::Vec<float, 3> cameraForward;
    sc::utils::Vec<float, 3> cameraRight;
    sc::utils::Vec<float, 3> cameraUp;
    float cameraLen;
    sc::utils::Vec<float, 2> cameraSize;
    sc::utils::Vec<float, 2> cameraRes;
    std::vector<sc::utils::Vec<float, 2>> roi;

    Snapshot(const sc::Camera<float, sc::VecArray>& camera,
                        std::vector<sc::utils::Vec<float, 2>>&& roi_) {
        roi = std::move(roi_);
        cameraPosition = camera.pos();
        cameraForward = mrc::getForward(camera.rot());
        cameraRight = mrc::getRight(cameraForward);
        cameraUp = mrc::getUp(cameraForward, cameraRight);
        cameraLen = camera.len();
        cameraSize = camera.size();
        cameraRes = camera.res();
    }
};

inline std::vector<mrc::Model<float>> snapshotsToModels(
    const std::vector<Snapshot>& snapshots)
{
    std::vector<mrc::Model<float>> models;
    models.reserve(snapshots.size());
    for (const auto& snap : snapshots)
    {
        mrc::Model<float> model;
        size_t n = snap.roi.size();
        if (n < 3) continue;
        std::vector<sc::utils::Vec<float, 3>> vertices(n + 1);
        std::vector<mrc::Model<float>::Face> faces(n + (n - 2));
        vertices[0] = snap.cameraPosition;
        for (size_t i = 0; i < snap.roi.size(); ++i)
        {
            sc::utils::Vec<float, 2> ndc{
                            (snap.roi[i][0] / snap.cameraRes[0]) * 2.f - 1.f,
                            1.f - (snap.roi[i][1] / snap.cameraRes[1]) * 2.f
                        };
            sc::utils::Vec<float, 2> camPC = ndc * (snap.cameraSize / 2.f);
            sc::utils::Vec<float, 3> camCord{camPC[0], camPC[1], snap.cameraLen};
            camCord = sc::utils::norm(camCord) * 100.f;
            sc::utils::Vec<float, 3> ws = snap.cameraPosition;
            ws += snap.cameraForward * camCord[2];
            ws += snap.cameraRight * camCord[0];
            ws += snap.cameraUp * camCord[1];
            vertices[i + 1] = ws;
        }
        for (size_t i = 0; i < n; ++i)
        {
            size_t currentBaseIdx = i + 1;
            size_t nextBaseIdx = (i == n - 1) ? 1 : i + 2;
            mrc::Model<float>::Face face;
            face[0] = {0, 0, 0};
            face[1] = {currentBaseIdx, 0, 0};
            face[2] = {nextBaseIdx, 0, 0};
            faces.push_back(face);
        }
        for (size_t i = 1; i < n - 1; ++i)
        {
            mrc::Model<float>::Face baseFace;
            baseFace[0] = {1, 0, 0};
            baseFace[1] = {i + 2, 0, 0};
            baseFace[2] = {i + 1, 0, 0};
            faces.push_back(baseFace);
        }
        model.verticies() = vertices;
        model.faces() = faces;
        models.push_back(model);
    }
    return models;
}