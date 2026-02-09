#pragma once

#include "vec.h"

namespace sc::utils
{

template<
    typename NumericT,
    std::size_t N,
    typename Container = std::array<NumericT, N>
>
class Ray
{
public:
    Ray() = default;
    ~Ray() = default;

    Ray(const Vec<NumericT, N, Container>& pos, const Vec<NumericT, N, Container>& rot)
        : _pos(pos)
        , _rot(rot)
    {}

    Ray(const Ray&) = default;
    Ray(Ray&&) = delete;
    Ray& operator=(const Ray&) = default;
    Ray& operator=(const Ray&&) = delete;

    const Vec<NumericT, N, Container>& pos() const { return _pos; }
    const Vec<NumericT, N, Container>& rot() const { return _rot; }
    Vec<NumericT, N, Container>& pos() { return _pos; }
    Vec<NumericT, N, Container>& rot() { return _rot; }

private:
    Vec<NumericT, N, Container> _pos;
    Vec<NumericT, N, Container> _rot;
};

} // namespace sc::utils