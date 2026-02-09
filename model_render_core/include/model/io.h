#pragma once

#include "model.h"

#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <unordered_map>
#include <cstring>
#include <cstdlib>

// stb_image: define implementation here.
// NOTE: if io.h is included from multiple translation units in the same
// executable, move the two lines below into exactly ONE .cpp file and
// keep only the bare #include in this header.
#ifndef STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#endif
#include "thirdparty/stb_image.h"

namespace mrc::io
{



namespace detail
{

inline std::string extractDir(const char* path)
{
    std::string s(path);
    auto pos = s.find_last_of('/');
    if (pos == std::string::npos)
        pos = s.find_last_of('\\');
    if (pos == std::string::npos)
        return "";
    return s.substr(0, pos + 1);
}

inline std::string resolvePath(const std::string& baseDir, const std::string& path)
{
    if (!path.empty() && path[0] == '/')
        return path;
    return baseDir + path;
}

inline std::pair<const char*, std::size_t> mmapFile(const char* path)
{
    int fd = open(path, O_RDONLY);
    if (fd == -1)
        throw std::runtime_error(std::string("io: cannot open ") + path);

    off_t sz = lseek(fd, 0, SEEK_END);
    if (sz <= 0) { close(fd); throw std::runtime_error(std::string("io: empty file ") + path); }

    void* data = mmap(nullptr, static_cast<std::size_t>(sz), PROT_READ, MAP_PRIVATE, fd, 0);
    close(fd);
    if (data == MAP_FAILED)
        throw std::runtime_error(std::string("io: mmap failed ") + path);

    return { static_cast<const char*>(data), static_cast<std::size_t>(sz) };
}

inline void skipSpaces(const char*& p, const char* end)
{
    while (p < end && (*p == ' ' || *p == '\t')) ++p;
}

inline void skipLine(const char*& p, const char* end)
{
    while (p < end && *p != '\n') ++p;
    if (p < end) ++p;
}

inline std::string readRestOfLine(const char*& p, const char* end)
{
    skipSpaces(p, end);
    const char* start = p;
    while (p < end && *p != '\n' && *p != '\r')
        ++p;
    const char* e = p;
    while (e > start && (*(e - 1) == ' ' || *(e - 1) == '\t')) --e;
    if (p < end) ++p;
    return {start, e};
}

inline float readFloat(const char*& p, const char* /*end*/)
{
    return std::strtof(p, const_cast<char**>(&p));
}



template<typename NumericT>
struct MtlEntry {
    sc::utils::Vec<NumericT, 3> kd{1, 1, 1};
    sc::utils::Vec<NumericT, 3> ka{0, 0, 0};
    sc::utils::Vec<NumericT, 3> ks{0, 0, 0};
    NumericT ns = 0;
    std::string mapKd;
};

template<typename NumericT>
std::unordered_map<std::string, MtlEntry<NumericT>>
parseMtlFile(const char* path, const std::string& baseDir)
{
    std::unordered_map<std::string, MtlEntry<NumericT>> materials;

    auto [ptr, size] = mmapFile(path);
    const char* end = ptr + size;
    const char* p = ptr;

    std::string currentName;

    while (p < end)
    {
        while (p < end && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r'))
            ++p;
        if (p >= end) break;

        if (std::strncmp(p, "newmtl", 6) == 0 && (p[6] == ' ' || p[6] == '\t'))
        {
            p += 6;
            currentName = readRestOfLine(p, end);
            materials[currentName] = MtlEntry<NumericT>{};
        }
        else if (p[0] == 'K' && p[1] == 'd' && (p[2] == ' ' || p[2] == '\t'))
        {
            p += 2;
            auto& m = materials[currentName];
            m.kd[0] = readFloat(p, end); m.kd[1] = readFloat(p, end); m.kd[2] = readFloat(p, end);
            skipLine(p, end);
        }
        else if (p[0] == 'K' && p[1] == 'a' && (p[2] == ' ' || p[2] == '\t'))
        {
            p += 2;
            auto& m = materials[currentName];
            m.ka[0] = readFloat(p, end); m.ka[1] = readFloat(p, end); m.ka[2] = readFloat(p, end);
            skipLine(p, end);
        }
        else if (p[0] == 'K' && p[1] == 's' && (p[2] == ' ' || p[2] == '\t'))
        {
            p += 2;
            auto& m = materials[currentName];
            m.ks[0] = readFloat(p, end); m.ks[1] = readFloat(p, end); m.ks[2] = readFloat(p, end);
            skipLine(p, end);
        }
        else if (p[0] == 'N' && p[1] == 's' && (p[2] == ' ' || p[2] == '\t'))
        {
            p += 2;
            materials[currentName].ns = readFloat(p, end);
            skipLine(p, end);
        }
        else if (std::strncmp(p, "map_Kd", 6) == 0 && (p[6] == ' ' || p[6] == '\t'))
        {
            p += 6;
            std::string texPath = readRestOfLine(p, end);
            materials[currentName].mapKd = resolvePath(baseDir, texPath);
        }
        else
        {
            skipLine(p, end);
        }
    }

    munmap(const_cast<char*>(ptr), size);
    return materials;
}



template<typename NumericT>
std::shared_ptr<Texture<NumericT>> loadTexture(const std::string& path)
{
    if (path.empty()) return nullptr;

    int w, h, channels;
    unsigned char* data = stbi_load(path.c_str(), &w, &h, &channels, 3);
    if (!data) {
        std::cerr << "io: failed to load texture: " << path
                  << " (" << stbi_failure_reason() << ")\n";
        return nullptr;
    }

    auto tex = Texture<NumericT>::fromRawBytes(
        data, static_cast<std::size_t>(w), static_cast<std::size_t>(h));
    stbi_image_free(data);

    return std::make_shared<Texture<NumericT>>(std::move(tex));
}

template<typename NumericT>
Material<NumericT> buildMaterial(const MtlEntry<NumericT>& entry)
{
    Material<NumericT> mat;
    mat.baseColor = entry.kd;
    mat.ambient   = (entry.ka[0] + entry.ka[1] + entry.ka[2]) / NumericT(3);
    mat.specular  = (entry.ks[0] + entry.ks[1] + entry.ks[2]) / NumericT(3);
    mat.shininess = (entry.ns > 0) ? entry.ns : NumericT(32);
    mat.diffuseMap = loadTexture<NumericT>(entry.mapKd);
    return mat;
}



inline std::array<std::size_t, 3> parseFaceVertex(const char*& p, const char* /*end*/)
{
    constexpr std::size_t MISSING = SIZE_MAX;
    std::array<std::size_t, 3> r{MISSING, MISSING, MISSING};

    long v = std::strtol(p, const_cast<char**>(&p), 10);
    r[0] = static_cast<std::size_t>(v - 1);

    if (*p == '/') {
        ++p;
        if (*p != '/') {
            long vt = std::strtol(p, const_cast<char**>(&p), 10);
            r[1] = static_cast<std::size_t>(vt - 1);
        }
        if (*p == '/') {
            ++p;
            long vn = std::strtol(p, const_cast<char**>(&p), 10);
            r[2] = static_cast<std::size_t>(vn - 1);
        }
    }
    return r;
}

} // namespace detail



template<typename NumericT>
Model<NumericT> readFromObjFile(
    const char* path,
    const sc::utils::Vec<NumericT, 3>& pos = sc::utils::Vec<NumericT, 3>{0, 0, 0},
    const sc::utils::Vec<NumericT, 3>& rot = sc::utils::Vec<NumericT, 3>{0, 0, 0})
{
    using Face = typename ModelGeometry<NumericT>::Face;

    std::string baseDir = detail::extractDir(path);

    auto [ptr, size] = detail::mmapFile(path);
    const char* end = ptr + size;
    const char* p = ptr;

    std::vector<sc::utils::Vec<NumericT, 3>> vertices;
    std::vector<sc::utils::Vec<NumericT, 2>> uvs;
    std::vector<sc::utils::Vec<NumericT, 3>> normals;
    std::vector<Face> faces;

    std::string mtlLibPath;
    std::string activeMaterial;

    while (p < end)
    {
        while (p < end && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r'))
            ++p;
        if (p >= end) break;

        if (*p == '#')
        {
            detail::skipLine(p, end);
        }
        else if (std::strncmp(p, "mtllib", 6) == 0 && (p[6] == ' ' || p[6] == '\t'))
        {
            p += 6;
            mtlLibPath = detail::readRestOfLine(p, end);
        }
        else if (std::strncmp(p, "usemtl", 6) == 0 && (p[6] == ' ' || p[6] == '\t'))
        {
            p += 6;
            activeMaterial = detail::readRestOfLine(p, end);
        }
        else if (p[0] == 'v' && p[1] == 'n' && (p[2] == ' ' || p[2] == '\t'))
        {
            p += 2;
            NumericT x = detail::readFloat(p, end);
            NumericT y = detail::readFloat(p, end);
            NumericT z = detail::readFloat(p, end);
            normals.emplace_back(x, y, z);
            detail::skipLine(p, end);
        }
        else if (p[0] == 'v' && p[1] == 't' && (p[2] == ' ' || p[2] == '\t'))
        {
            p += 2;
            NumericT u = detail::readFloat(p, end);
            NumericT v = detail::readFloat(p, end);
            uvs.emplace_back(u, v);
            detail::skipLine(p, end);
        }
        else if (p[0] == 'v' && (p[1] == ' ' || p[1] == '\t'))
        {
            p += 1;
            NumericT x = detail::readFloat(p, end);
            NumericT y = detail::readFloat(p, end);
            NumericT z = detail::readFloat(p, end);
            vertices.emplace_back(x, y, z);
            detail::skipLine(p, end);
        }
        else if (p[0] == 'f' && (p[1] == ' ' || p[1] == '\t'))
        {
            p += 1;

            std::vector<std::array<std::size_t, 3>> faceVerts;
            while (p < end && *p != '\n' && *p != '\r')
            {
                detail::skipSpaces(p, end);
                if (p >= end || *p == '\n' || *p == '\r') break;
                faceVerts.push_back(detail::parseFaceVertex(p, end));
            }
            detail::skipLine(p, end);

            for (std::size_t i = 2; i < faceVerts.size(); ++i)
            {
                Face face{};
                face[0] = faceVerts[0];
                face[1] = faceVerts[i - 1];
                face[2] = faceVerts[i];
                faces.push_back(face);
            }
        }
        else
        {
            detail::skipLine(p, end);
        }
    }

    munmap(const_cast<char*>(ptr), size);

    ModelGeometry<NumericT> geometry;
    geometry.verticies() = std::move(vertices);
    geometry.uv()        = std::move(uvs);
    geometry.normals()   = std::move(normals);
    geometry.faces()     = std::move(faces);
    geometry.pos()       = pos;
    geometry.rot()       = rot;

    Material<NumericT> material;
    if (!mtlLibPath.empty())
    {
        std::string fullMtlPath = detail::resolvePath(baseDir, mtlLibPath);
        try {
            auto mtlMap = detail::parseMtlFile<NumericT>(fullMtlPath.c_str(), baseDir);
            const detail::MtlEntry<NumericT>* entry = nullptr;
            if (!activeMaterial.empty() && mtlMap.count(activeMaterial))
                entry = &mtlMap.at(activeMaterial);
            else if (!mtlMap.empty())
                entry = &mtlMap.begin()->second;
            if (entry)
                material = detail::buildMaterial(*entry);
        }
        catch (const std::exception& e) {
            std::cerr << "io: warning: could not load MTL: " << e.what() << "\n";
        }
    }

    return Model<NumericT>{ std::move(geometry), std::move(material) };
}

} // namespace mrc::io


namespace mrc {
    using io::readFromObjFile;
} // namespace mrc
