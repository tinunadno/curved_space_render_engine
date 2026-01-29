#pragma once

#include "vec.h"

#include <array>
#include <cassert>
#include <algorithm>

namespace sc::utils
{

template<
    typename NumericT,
    std::size_t Rows,
    std::size_t Cols,
    typename Container = std::array<NumericT, Rows * Cols>
>
class Mat
{
public:
    static constexpr std::size_t rows = Rows;
    static constexpr std::size_t cols = Cols;

    Mat() = default;

    explicit Mat(const Container& data)
        : _data(data)
    {}

    explicit Mat(Container&& data)
        : _data(std::move(data))
    {}

    /* Vec -> column matrix */
    explicit Mat(const Vec<NumericT, Rows>& v)
    {
        static_assert(Cols == 1);
        for (std::size_t i = 0; i < Rows; ++i)
            (*this)(i, 0) = v[i];
    }

    explicit Mat(Vec<NumericT, Rows>&& v)
    {
        static_assert(Cols == 1);
        for (std::size_t i = 0; i < Rows; ++i)
            (*this)(i, 0) = std::move(v[i]);
    }

    template<
        typename... Args,
        typename = std::enable_if_t<
            sizeof...(Args) == Rows * Cols &&
            (std::is_convertible_v<Args, NumericT> && ...)
        >
    >
    explicit Mat(Args&&... args)
        : _data{ static_cast<NumericT>(std::forward<Args>(args))... }
    {}

    static Mat zeros() {
        Mat<NumericT, Rows, Cols> r;
        for (std::size_t i = 0; i < Rows * Cols; ++i)
            r._data[i] = NumericT(0);
        return r;
    }

    static Mat identity() {
        Mat<NumericT, Rows, Cols> r;
        for (std::size_t i = 0; i < Rows * Cols; ++i)
            r._data[i] = NumericT(0);
        for (std::size_t i = 0; i < std::min(Rows, Cols); ++i)
            r._data[i] = NumericT(1);
        return r;
    }

    Mat t() const {
        Mat<NumericT, Cols, Rows> transposed;
        for (std::size_t i = 0; i < Rows; ++i) {
            for (std::size_t j = 0; j < Cols; ++j) {
                transposed(j, i) = (*this)(i, j);
            }
        }
        return transposed;
    }

    NumericT& operator()(std::size_t r, std::size_t c)
    {
#ifndef NDEBUG
        assert(r < Rows && c < Cols);
#endif
        return _data[r * Cols + c];
    }

    const NumericT& operator()(std::size_t r, std::size_t c) const
    {
#ifndef NDEBUG
        assert(r < Rows && c < Cols);
#endif
        return _data[r * Cols + c];
    }

    Vec<NumericT, Rows * Cols> operator*() const {
        return Vec<NumericT, Rows * Cols>{_data};
    }

    auto begin() { return _data.begin(); }
    auto end() { return _data.end(); }
    auto begin() const { return _data.begin(); }
    auto end() const { return _data.end(); }

private:
    Container _data{};
};

template<typename T, std::size_t R, std::size_t C>
Mat<T, R, C> operator+(const Mat<T, R, C>& a, const Mat<T, R, C>& b)
{
    Mat<T, R, C> r;
    for (std::size_t i = 0; i < R * C; ++i)
        r.begin()[i] = a.begin()[i] + b.begin()[i];
    return r;
}

template<typename T, std::size_t R, std::size_t C>
Mat<T, R, C> operator-(const Mat<T, R, C>& a, const Mat<T, R, C>& b)
{
    Mat<T, R, C> r;
    for (std::size_t i = 0; i < R * C; ++i)
        r.begin()[i] = a.begin()[i] - b.begin()[i];
    return r;
}

template<typename T, std::size_t R, std::size_t K>
Mat<T, R, K> operator-(const Mat<T, R, K>& a)
{
    Mat<T, R, K> r{a};
    for (auto& v : r)
    {
        v *= -1;
    }
    return r;
}

template<
    typename T,
    std::size_t R,
    std::size_t K,
    std::size_t C
>
Mat<T, R, C> operator*(const Mat<T, R, K>& a, const Mat<T, K, C>& b)
{
    Mat<T, R, C> r;

    for (std::size_t i = 0; i < R; ++i)
        for (std::size_t j = 0; j < C; ++j)
        {
            T sum{};
            for (std::size_t k = 0; k < K; ++k)
                sum += a(i, k) * b(k, j);
            r(i, j) = sum;
        }
    return r;
}

template<typename T, std::size_t R, std::size_t C>
Vec<T, R> operator*(const Mat<T, R, C>& m, const Vec<T, C>& v)
{
    Vec<T, R> r;

    for (std::size_t i = 0; i < R; ++i)
    {
        T sum{};
        for (std::size_t j = 0; j < C; ++j)
            sum += m(i, j) * v[j];
        r[i] = sum;
    }
    return r;
}

template<typename T, std::size_t R, std::size_t C>
Vec<T, C> operator*(const Vec<T, R>& v, const Mat<T, R, C>& m)
{
    Vec<T, C> r;

    for (std::size_t j = 0; j < C; ++j)
    {
        T sum{};
        for (std::size_t i = 0; i < R; ++i)
            sum += v[i] * m(i, j);
        r[j] = sum;
    }
    return r;
}

} // namespace sc::utils
