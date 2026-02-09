#pragma once

#include "utils/vec.h"
#include "camera_view.h"

namespace sc
{

template<
    typename NumericT,
    template<typename, std::size_t> typename VecT
>
class Camera
{
public:

    using Vec2 = VecT<NumericT, 2>;
    using Vec3 = VecT<NumericT, 3>;

    Camera()
        : _pos()
        , _rot()
        , _res(800, 600)
        , _size(.8, -.1) // second one 'gonna be scaled further according to ratio
        , _len(.05)
    {
        setRatioFromRes();
    }

    Camera(const Camera<NumericT, VecT>&) = default;
    Camera(Camera<NumericT, VecT>&&) = delete;
    Camera& operator=(const Camera<NumericT, VecT>&) = default;
    Camera& operator=(const Camera<NumericT, VecT>&&) = delete;

    ~Camera() = default;

    // Getters
    const auto& pos() const { return _pos; }
    const auto& rot() const { return _rot; }
    const auto& res() const { return _res; }
    const auto& size() const { return _size; }
    NumericT len() const { return _len; }
    NumericT sideRatio() const { return _res[0] / _res[1]; }

    // Setters
    auto& rot() { return _rot; }
    auto& pos() { return _pos; }

    void setRes(const Vec2& res)
    {
        _res = res;
        setRatioFromRes();
    }

    void setResX(NumericT res)
    {
        _res[0] = res;
        setRatioFromRes();
    }

    void setResY(NumericT res)
    {
        _res[1] = res;
        setRatioFromRes();
    }

    void setSize(const Vec2& sz)
    {
        _size = sz;
        setRatioFromSize();
    }

    void setSizeX(NumericT sz)
    {
        _size[0] = sz;
        setRatioFromSize();
    }

    void setSizeY(NumericT sz)
    {
        _size[1] = sz;
        setRatioFromSize();
    }

    void setLen(NumericT len)
    {
        _len = len;
    }

private:

    void setRatioFromRes()
    {
        NumericT sideRatio = _res[0] / _res[1];
        _size[1] = _size[0] / sideRatio;
    }

    void setRatioFromSize()
    {
        NumericT sideRatio = _size[0] / _size[1];
        _res[1] = _res[0] / sideRatio;
    }

    /// \var Vec3 _pos camera center position in meters
    Vec3 _pos;
    /// \var Vec3 _rot camera rotation in radians
    Vec3 _rot;
    /// \var Vec2 _res camera resolution in pixels
    Vec2 _res;
    /// \var Vec2 _size camera screen size in meters
    Vec2 _size;
    /// \var NumericT _len camera focal length in meters
    NumericT _len;
};

template<
    typename NumericT,
    template<typename, std::size_t> typename VecT
>
internal::CameraView<NumericT>
makeViewFromCamera(const Camera<NumericT, VecT>& camera)
{
    using Vec3 = utils::Vec<NumericT, 3>;
    using Vec2 = utils::Vec<NumericT, 2>;

    internal::CameraView<NumericT> view;

    view.origin = camera.pos();

    // base axis
    Vec3 forward{ 0, 0, -1 };
    Vec3 right{ 1, 0,  0 };
    Vec3 up{ 0, 1,  0 };

    // camera rotation
    forward = rotateEuler(forward, camera.rot());
    right = rotateEuler(right, camera.rot());
    up = cross(right, forward);

    view.forward = forward;
    view.right = right;
    view.up = up;

    const Vec2 res = camera.res();
    const Vec2 size = camera.size();

    view.width = static_cast<std::size_t>(res[0]);
    view.height = static_cast<std::size_t>(res[1]);

    const NumericT pixelW = size[0] / static_cast<NumericT>(view.width);
    const NumericT pixelH = size[1] / static_cast<NumericT>(view.height);

    view.stepX = scalarBinaryOp(right, pixelW, std::multiplies<>{});
    view.stepY = scalarBinaryOp(up, pixelH, std::multiplies<>{});

    // upper left screen angle
    auto fwdScaled = scalarBinaryOp(forward, camera.len(), std::multiplies<>{});

    // right * (sizeX / 2)
    auto rightScaled = scalarBinaryOp(right, size[0] * 0.5f, std::multiplies<>{});

    // up * (sizeY / 2)
    auto upScaled = scalarBinaryOp(up, size[1] * 0.5f, std::multiplies<>{});

    // fwd_scaled - right_scaled
    auto temp = binaryOp(fwdScaled, rightScaled, [](auto x, auto y){ return x - y; });

    // temp - up_scaled => pixelOrigin
    view.pixelOrigin = binaryOp(temp, upScaled, [](auto x, auto y){ return x - y; });

    return view;
}

} // namespace sc