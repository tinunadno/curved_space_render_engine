#pragma once

#include "utils/vec.h"

#include <fstream>
#include <sys/mman.h>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>
namespace mrc {

template<typename NumericT>
class Model
{
public:

    using Face = std::array<std::array<std::size_t, 3>, 3>;

    Model():
        _verticies(),
        _uv(),
        _normals(),
        _faces(),
        _pos(),
        _rot()
    {}

    ~Model() = default;

    Model(const Model& model):
        _verticies(model._verticies),
        _uv(model._uv),
        _normals(model._normals),
        _faces(model._faces),
        _pos(model._pos),
        _rot(model._rot)
    {}

    Model(Model&& model) noexcept:
        _verticies(std::move(model._verticies)),
        _uv(std::move(model._uv)),
        _normals(std::move(model._normals)),
        _faces(std::move(model._faces)),
        _pos(std::move(model._pos)),
        _rot(std::move(model._rot))
    {}

    Model& operator=(const Model& model) = default;

    const std::vector<sc::utils::Vec<NumericT, 3>>& verticies() const {return _verticies;}
    const std::vector<sc::utils::Vec<NumericT, 2>>& uv() const {return _uv;}
    const std::vector<sc::utils::Vec<NumericT, 3>>& normals() const {return _normals;}
    [[nodiscard]] const std::vector<Face>& faces() const {return _faces;}
    const sc::utils::Vec<NumericT, 3>& pos() const {return _pos;}
    const sc::utils::Vec<NumericT, 3>& rot() const {return _rot;}

    std::vector<sc::utils::Vec<NumericT, 3>>& verticies() {return _verticies;}
    std::vector<sc::utils::Vec<NumericT, 2>>& uv() {return _uv;}
    std::vector<sc::utils::Vec<NumericT, 3>>& normals() {return _normals;}
    [[nodiscard]] std::vector<Face>& faces() {return _faces;}
    sc::utils::Vec<NumericT, 3>& pos() {return _pos;}
    sc::utils::Vec<NumericT, 3>& rot() {return _rot;}

    std::array<sc::utils::Vec<NumericT, 3>, 3> getPolygon(std::size_t faceIdx,
        const std::vector<sc::utils::Vec<NumericT, 3>>& vertSource) const {
        return {vertSource[_faces[faceIdx][0][0]],
                vertSource[_faces[faceIdx][1][0]],
                vertSource[_faces[faceIdx][2][0]]};
    }

private:


    std::vector<sc::utils::Vec<NumericT, 3>> _verticies;
    std::vector<sc::utils::Vec<NumericT, 2>> _uv;
    std::vector<sc::utils::Vec<NumericT, 3>> _normals;
    std::vector<Face> _faces;

    sc::utils::Vec<NumericT, 3> _pos;
    sc::utils::Vec<NumericT, 3> _rot;
};

template<typename NumericT>
Model<NumericT> readFromObjFile(
    const char* path,
    const sc::utils::Vec<NumericT, 3>& pos = sc::utils::Vec<NumericT, 3>{0., 0., 0.},
    const sc::utils::Vec<NumericT, 3>& rot = sc::utils::Vec<NumericT, 3>{0., 0., 0.})
{
    std::string reason;

    int fd = open(path, O_RDONLY);
    if (fd == -1) {
        reason = "failed to open file";
        std::cerr << "Failed to load OBJ: " << path << "\n\t" << reason << "\n";
        throw std::runtime_error(reason);
    }

    off_t size = lseek(fd, 0, SEEK_END);
    if (size <= 0) {
        close(fd);
        reason = "file is empty";
        std::cerr << "Failed to load OBJ: " << path << "\n\t" << reason << "\n";
        throw std::runtime_error(reason);
    }

    void* data = mmap(nullptr, size, PROT_READ, MAP_PRIVATE, fd, 0);
    close(fd);
    if (data == MAP_FAILED) {
        reason = "mmap failed";
        std::cerr << "Failed to load OBJ: " << path << "\n\t" << reason << "\n";
        throw std::runtime_error(reason);
    }

    const char* ptr = static_cast<const char*>(data);
    const char* end = ptr + size;

    std::vector<sc::utils::Vec<NumericT, 3>> verticies;
    std::vector<sc::utils::Vec<NumericT, 2>> uv;
    std::vector<sc::utils::Vec<NumericT, 3>> normals;
    std::vector<typename Model<NumericT>::Face> faces;

    while (ptr < end) {
        while (ptr < end && (*ptr == ' ' || *ptr == '\n' || *ptr == '\r' || *ptr == '\t'))
            ++ptr;

        if (ptr >= end)
            break;

        if (*ptr == 'v' && *(ptr + 1) == ' ') {
            ptr += 2;
            NumericT x = std::strtod(ptr, const_cast<char**>(&ptr));
            NumericT y = std::strtod(ptr, const_cast<char**>(&ptr));
            NumericT z = std::strtod(ptr, const_cast<char**>(&ptr));
            verticies.emplace_back(x, y, z);
        }

        else if (*ptr == 'v' && *(ptr + 1) == 't') {
            ptr += 3;
            NumericT u = std::strtod(ptr, const_cast<char**>(&ptr));
            NumericT v = std::strtod(ptr, const_cast<char**>(&ptr));
            uv.emplace_back(u, v);
        }

        else if (*ptr == 'v' && *(ptr + 1) == 'n') {
            ptr += 3;
            NumericT x = std::strtod(ptr, const_cast<char**>(&ptr));
            NumericT y = std::strtod(ptr, const_cast<char**>(&ptr));
            NumericT z = std::strtod(ptr, const_cast<char**>(&ptr));
            normals.emplace_back(x, y, z);
        }

        else if (*ptr == 'f' && *(ptr + 1) == ' ') {
            ptr += 2;

            typename Model<NumericT>::Face face{};

            for (int i = 0; i < 3; ++i) {

                face[i][0] = std::strtol(ptr, const_cast<char**>(&ptr), 10) - 1;
                ++ptr; // skip '/'

                face[i][1] = std::strtol(ptr, const_cast<char**>(&ptr), 10) - 1;
                ++ptr; // skip '/'

                face[i][2] = std::strtol(ptr, const_cast<char**>(&ptr), 10) - 1;
            }

            faces.push_back(face);
        }

        // skip rest of line
        while (ptr < end && *ptr != '\n')
            ++ptr;
    }

    munmap(data, size);

    Model<NumericT> model;
    model.verticies() = std::move(verticies);
    model.uv() = std::move(uv);
    model.normals() = std::move(normals);
    model.faces() = std::move(faces);
    model.pos() = pos;
    model.rot() = rot;

    return model;
}


} // namespace mrc