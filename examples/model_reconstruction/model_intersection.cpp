#include "model_intersection.h"
#include <iostream>


manifold::MeshGL mi::ObjectToMeshGL(const mrc::Model<float>& obj) {
    manifold::MeshGL mesh;
    if (obj.verticies().empty()) return mesh;

    mesh.numProp = 3;
    mesh.vertProperties.resize(obj.verticies().size() * 3);

    for (size_t i = 0; i < obj.verticies().size(); ++i) {
        mesh.vertProperties[i * 3 + 0] = obj.verticies()[i][0];
        mesh.vertProperties[i * 3 + 1] = obj.verticies()[i][1];
        mesh.vertProperties[i * 3 + 2] = obj.verticies()[i][2];
    }

    for (const auto face : obj.faces()) {
        mesh.triVerts.push_back(static_cast<uint32_t>(face[0][0]));
        mesh.triVerts.push_back(static_cast<uint32_t>(face[1][0]));
        mesh.triVerts.push_back(static_cast<uint32_t>(face[2][0]));
    }

    return mesh;
}

mrc::Model<float> mi::MeshGLToObject(const manifold::MeshGL& mesh) {
    mrc::Model<float> obj;

    size_t numVerts = mesh.vertProperties.size() / mesh.numProp;

    std::vector<sc::utils::Vec<float, 3>> vertices;
    std::vector<mrc::Model<float>::Face> faces;
    faces.resize(mesh.triVerts.size());

    vertices.resize(numVerts);
    for (size_t i = 0; i < numVerts; ++i) {
        vertices[i] = sc::utils::Vec<float, 3>{
            mesh.vertProperties[i * mesh.numProp + 0],
            mesh.vertProperties[i * mesh.numProp + 1],
            mesh.vertProperties[i * mesh.numProp + 2]
        };
    }

    for (size_t f = 0; f < mesh.triVerts.size(); f += 3) {
        faces[f][0] = {mesh.triVerts[f], 0, 0};
        faces[f][1] = {mesh.triVerts[f + 1], 0, 0};
        faces[f][2] = {mesh.triVerts[f + 2], 0, 0};
    }

    obj.verticies() = vertices;
    obj.faces() = faces;

    return obj;
}

mrc::Model<float> mi::ComputeIntersection(const std::vector<mrc::Model<float>>& inputObjects) {
    if (inputObjects.empty()) {
        return {};
    }

    if (inputObjects.size() == 1) {
        return inputObjects[0];
    }

    std::vector<manifold::Manifold> manifolds;
    manifolds.reserve(inputObjects.size());

    for (size_t i = 0; i < inputObjects.size(); ++i) {
        manifold::MeshGL mesh = ObjectToMeshGL(inputObjects[i]);
        manifold::Manifold m(mesh);

        // if there are some invalid meshes, manifold'll return empty object
        if (m.Status() != manifold::Manifold::Error::NoError) {
            std::cerr << "Warning: Object at index " << i << " is invalid. Skipping. "
                      << "Error code: " << static_cast<int>(m.Status()) << std::endl;
            continue;
        }

        // checking if object is *whater-proof hehe*
        if (m.IsEmpty()) {
            std::cerr << "Warning: Object at index " << i << " resulted in empty manifold." << std::endl;
            continue;
        }

        manifolds.push_back(std::move(m));
    }

    // after validation here might be nothing
    if (manifolds.empty()) return {};
    if (manifolds.size() == 1) return MeshGLToObject(manifolds[0].GetMeshGL());

    try {
        manifold::Manifold result = manifold::Manifold::BatchBoolean(manifolds, manifold::OpType::Intersect);

        // and checking the result
        if (result.IsEmpty()) {
            std::cout << "Info: Intersection is empty (objects do not overlap)." << std::endl;
            return {};
        }

        return MeshGLToObject(result.GetMeshGL());

    } catch (const std::exception& e) {
        std::cerr << "Critical error during Boolean operation: " << e.what() << std::endl;
        return {};
    }
}