#pragma once
#include <utils/vec.h>

#include <algorithm>
#include <cmath>
#include <vector>
#include <memory>

namespace mrc
{



namespace texture_internals
{

struct Byte255NormalizingPolicy {
    sc::utils::Vec<float, 3> normalize(const sc::utils::Vec<float, 3>& col) const { return col / 255.f; }
};

struct IdentityNormalizingPolicy {
    sc::utils::Vec<float, 3> normalize(const sc::utils::Vec<float, 3>& col) const { return col; }
};

template<typename PixelType>
struct RGBStoragePolicy {
    sc::utils::Vec<float, 3> toRgb(PixelType r, PixelType g, PixelType b) const {
        return sc::utils::Vec<float, 3>{
            static_cast<float>(r),
            static_cast<float>(g),
            static_cast<float>(b),
        };
    }
};

template<typename PixelType>
struct BGRStoragePolicy {
    sc::utils::Vec<float, 3> toRgb(PixelType b, PixelType g, PixelType r) const {
        return sc::utils::Vec<float, 3>{
            static_cast<float>(r),
            static_cast<float>(g),
            static_cast<float>(b),
        };
    }
};

} // namespace texture_internals



template<typename NumericT>
class Texture
{
public:
    Texture(std::vector<sc::utils::Vec<float, 3>>&& pixels, std::size_t w, std::size_t h)
        : _pixels(std::move(pixels)), _width(w), _height(h) {}

    /// Bilinear sampling.  UV is in [0,1] with wrap (repeat).
    sc::utils::Vec<float, 3> sample(const sc::utils::Vec<NumericT, 2>& uv) const
    {
        if (_width == 0 || _height == 0)
            return sc::utils::Vec<float, 3>{0, 0, 0};

        NumericT u = uv[0] - std::floor(uv[0]);
        NumericT v = uv[1] - std::floor(uv[1]);

        NumericT fx = u * static_cast<NumericT>(_width  - 1);
        NumericT fy = v * static_cast<NumericT>(_height - 1);

        auto x0 = static_cast<std::size_t>(fx);
        auto y0 = static_cast<std::size_t>(fy);
        std::size_t x1 = std::min(x0 + 1, _width  - 1);
        std::size_t y1 = std::min(y0 + 1, _height - 1);

        NumericT tx = fx - static_cast<NumericT>(x0);
        NumericT ty = fy - static_cast<NumericT>(y0);

        const auto& c00 = _pixels[y0 * _width + x0];
        const auto& c10 = _pixels[y0 * _width + x1];
        const auto& c01 = _pixels[y1 * _width + x0];
        const auto& c11 = _pixels[y1 * _width + x1];

        float ftx = static_cast<float>(tx);
        float fty = static_cast<float>(ty);

        return c00 * (1.f - ftx) * (1.f - fty) + c10 * ftx * (1.f - fty)
             + c01 * (1.f - ftx) * fty          + c11 * ftx * fty;
    }

    [[nodiscard]] std::size_t width()  const { return _width;  }
    [[nodiscard]] std::size_t height() const { return _height; }



    /// Create from raw RGB unsigned-char data (3 bytes per pixel, 0-255).
    static Texture fromRawBytes(const unsigned char* data, std::size_t w, std::size_t h)
    {
        std::vector<sc::utils::Vec<float, 3>> pixels(w * h);
        for (std::size_t i = 0; i < w * h; ++i) {
            pixels[i] = sc::utils::Vec<float, 3>{
                static_cast<float>(data[i * 3 + 0]) / 255.f,
                static_cast<float>(data[i * 3 + 1]) / 255.f,
                static_cast<float>(data[i * 3 + 2]) / 255.f
            };
        }
        return Texture(std::move(pixels), w, h);
    }

    /// Create from any raw data using NormalizingPolicy + ColorStoragePolicy.
    /// Accessor must support operator[](std::size_t) returning raw channel values.
    template<typename Accessor, typename NormP, typename ColP>
    static Texture fromRaw(const Accessor& data, NormP normP, ColP colP,
                           std::size_t w, std::size_t h)
    {
        std::vector<sc::utils::Vec<float, 3>> pixels(w * h);
        for (std::size_t i = 0; i < w * h; ++i) {
            std::size_t base = i * 3;
            pixels[i] = normP.normalize(
                colP.toRgb(data[base], data[base + 1], data[base + 2])
            );
        }
        return Texture(std::move(pixels), w, h);
    }

private:
    std::vector<sc::utils::Vec<float, 3>> _pixels;
    std::size_t _width;
    std::size_t _height;
};

} // namespace mrc
