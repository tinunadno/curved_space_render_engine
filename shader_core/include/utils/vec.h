#pragma once

#include <cstddef>
#include <algorithm>
#include <array>
#include <complex>

namespace sc::utils
{

template<
    typename NumericT,
    std::size_t N,
    typename Container = std::array<NumericT, N>
>
class Vec
{
public:

    Vec() = default;

    explicit Vec(const Container& data)
        : _data(data)
    {}

    explicit Vec(Container&& data)
        : _data(std::move(data))
    {}

    template<
        typename... Args,
        typename = std::enable_if<
            sizeof...(Args) == N &&
            (std::is_convertible_v<Args, NumericT> && ...)
        >
    >
    explicit Vec(Args&&... args)
        : _data{ static_cast<NumericT>(std::forward<Args>(args))... }
    {}

    template<typename OtherContainer>
    Vec(const Vec<NumericT, N, OtherContainer>& vec)
        : _data(N)
    {
        std::copy(vec.begin(), vec.end(), begin());
    }

    template<typename OtherNumericT>
    Vec(const Vec<OtherNumericT, N, Container>& vec)
        : _data(N)
    {
        std::transform(vec.begin(), vec.end(), begin(),
            [](const OtherNumericT& val)
            {
               return static_cast<NumericT>(val);
            });
    }

    NumericT& operator[](std::size_t ind) { return _data[ind]; }

    const NumericT& operator[](std::size_t ind) const { return _data[ind]; }

    auto begin() { return _data.begin(); }
    auto end() { return _data.end(); }
    auto begin() const { return _data.begin(); }
    auto end() const { return _data.end(); }

    NumericT* data() { return _data.data(); }
    const NumericT* data() const { return _data.data(); }

private:
    Container _data;
};

template<typename NumericT, std::size_t N, typename Container, typename Op>
constexpr Vec<NumericT, N, Container>
binaryOp(
    const Vec<NumericT, N, Container>& a
    , const Vec<NumericT, N, Container>& b
    , Op op)
{
    Vec<NumericT, N, Container> ret;
    for (std::size_t i = 0; i < N; i++)
    {
        ret[i] = op(a[i], b[i]);
    }
    return ret;
}

template<typename NumericT, std::size_t N, typename Container, typename Op>
constexpr Vec<NumericT, N, Container>
unaryOp(
    const Vec<NumericT, N, Container>& a
    , Op op)
{
    Vec<NumericT, N, Container> ret;
    for (std::size_t i = 0; i < N; i++)
    {
        ret[i] = op(a[i]);
    }
    return ret;
}

template<typename NumericT, std::size_t N, typename Container, typename Op>
constexpr Vec<NumericT, N, Container>
scalarBinaryOp(
    const Vec<NumericT, N, Container>& a
    , NumericT scalar
    , Op op)
{
    Vec<NumericT, N, Container> ret;
    for (std::size_t i = 0; i < N; i++)
    {
        ret[i] = op(a[i], scalar);
    }
    return ret;
}

template<typename NumericT, std::size_t N, typename Container>
constexpr void
operator+=(
    Vec<NumericT, N, Container>& a,
    const Vec<NumericT, N, Container>& b)
{
    for (std::size_t i = 0; i < N; i++)
    {
        a[i] += b[i];
    }
}

template<typename NumericT, std::size_t N, typename Container>
constexpr Vec<NumericT, N, Container>
operator*(const Vec<NumericT, N, Container>& a
    , NumericT scalar)
{
    return scalarBinaryOp(a, scalar, std::multiplies<>{});
}

template<typename NumericT, std::size_t N, typename Container>
constexpr Vec<NumericT, N, Container>
operator*(const Vec<NumericT, N, Container>& a
    , const Vec<NumericT, N, Container>& b)
{
    return binaryOp(a, b, std::multiplies<>{});
}

template<typename NumericT, std::size_t N, typename Container>
constexpr Vec<NumericT, N, Container>
operator/(const Vec<NumericT, N, Container>& a
    , const Vec<NumericT, N, Container>& b)
{
    return binaryOp(a, b, [](const auto a, const auto b) { return a / b; });
}

template<typename NumericT, std::size_t N, typename Container>
constexpr Vec<NumericT, N, Container>
operator*(NumericT scalar
        , const Vec<NumericT, N, Container>& a
)
{
    return scalarBinaryOp(a, scalar, std::multiplies<>{});
}

template<typename NumericT, std::size_t N, typename Container>
constexpr Vec<NumericT, N, Container>
operator/(const Vec<NumericT, N, Container>& a
    , NumericT scalar)
{
    return scalarBinaryOp(a, scalar, [](const auto aa, const auto bb) { return aa / bb; });
}

template<typename NumericT, std::size_t N, typename Container>
constexpr Vec<NumericT, N, Container>
operator+(const Vec<NumericT, N, Container>& a
        , const Vec<NumericT, N, Container>& b)
{
    return binaryOp(a, b, [](const auto aa, const auto bb) { return aa + bb; });
}

template<typename NumericT, std::size_t N, typename Container>
constexpr Vec<NumericT, N, Container>
operator+(const Vec<NumericT, N, Container>& a
            , NumericT scalar)
{
    return scalarBinaryOp(a, scalar, [](const auto aa, const auto bb) { return aa + bb; });
}


template<typename NumericT, std::size_t N, typename Container>
constexpr Vec<NumericT, N, Container>
operator-(const Vec<NumericT, N, Container>& a)
{
    return unaryOp(a, [](const auto a) { return -a; });
}

template<typename NumericT, std::size_t N, typename Container>
constexpr Vec<NumericT, N, Container>
operator-(const Vec<NumericT, N, Container>& a
        , const Vec<NumericT, N, Container>& b)
{
    return binaryOp(a, b, [](const auto aa, const auto bb) { return aa - bb; });
}

template<typename NumericT, std::size_t N, typename Container>
constexpr Vec<NumericT, N, Container>
clamp(const Vec<NumericT, N, Container>& a
        , NumericT min
        , NumericT max)
{
    return unaryOp(a, [&min, &max](const auto val) { return std::max(min, std::min(max, val)); });
}

template<typename NumericT, std::size_t N, typename Container>
constexpr NumericT
dot(
    const Vec<NumericT, N, Container>& a
    , const Vec<NumericT, N, Container>& b)
{
    NumericT sum = 0;
    for (std::size_t i = 0; i < N; i++)
    {
        sum += a[i] * b[i];
    }
    return sum;
}

template<typename T, typename C>
constexpr Vec<T, 3, C> cross(const Vec<T, 3, C>& a, const Vec<T, 3, C>& b)
{
    return Vec<T, 3, C>{
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    };
}

template<typename NumericT, std::size_t N, typename Container>
constexpr NumericT
len(
    const Vec<NumericT, N, Container>& vec)
{
    return std::sqrt(dot(vec, vec));
}

template<typename NumericT, std::size_t N, typename Container>
constexpr NumericT
distance(
    const Vec<NumericT, N, Container>& a,
    const Vec<NumericT, N, Container>& b)
{
    return len(b - a);
}

template<typename NumericT, std::size_t N, typename Container>
constexpr Vec<NumericT, N, Container>
norm(
    const Vec<NumericT, N, Container>& vec)
{
    return scalarBinaryOp(vec, len(vec), [](NumericT a, NumericT b) { return a / b; });
}

template<typename NumericT, std::size_t N, typename Container>
Vec<NumericT, N, Container> rotateEuler(const Vec<NumericT, N, Container>& v, const Vec<NumericT, N, Container>& euler)
{
    NumericT cx = std::cos(euler[0]);
    NumericT sx = std::sin(euler[0]);
    NumericT cy = std::cos(euler[1]);
    NumericT sy = std::sin(euler[1]);
    NumericT cz = std::cos(euler[2]);
    NumericT sz = std::sin(euler[2]);

    Vec<NumericT, N, Container> v1{
        v[0],
        cx*v[1] - sx*v[2],
        sx*v[1] + cx*v[2]
    };

    Vec<NumericT, N, Container> v2{
        cy*v1[0] + sy*v1[2],
        v1[1],
        -sy*v1[0] + cy*v1[2]
    };

    Vec<NumericT, N, Container> v3{
        cz*v2[0] - sz*v2[1],
        sz*v2[0] + cz*v2[1],
        v2[2]
    };

    return v3;
}

template<typename T, typename C>
Vec<T, 3, C>
rotateAroundAxis(
        const Vec<T, 3, C>& v,
        const Vec<T, 3, C>& axis,
        T angle)
{
    const T c = std::cos(angle);
    const T s = std::sin(angle);

    Vec<T, 3, C> result;

    auto crossProduct = cross(axis, v);
    T dotProduct = dot(axis, v);

    for (int i = 0; i < 3; ++i)
    {
        result[i] =
            v[i] * c +
            crossProduct[i] * s +
            axis[i] * dotProduct * (1 - c);
    }

    return result;
}

template<typename T, typename C>
T getCos(const Vec<T, 3, C>& a,
        const Vec<T, 3, C>& b)
{
    return dot(a, b) / (len(a) * len(b));
}

} // namespace sc::utils