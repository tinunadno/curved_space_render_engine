#pragma once

#include "camera_view.h"

namespace sc
{

template<typename NumericT>
class internal::CameraView<NumericT>::Iterator
{
public:
    Iterator(const CameraView* view, std::size_t index)
        : _view(view), _totalPixels(view->width * view->height)
    {
        _uvStepX = NumericT(2) / static_cast<NumericT>(view->width);
        _uvStepY = NumericT(2) / static_cast<NumericT>(view->height);

        if (index >= _totalPixels) {
            _index = _totalPixels;
            return;
        }

        _index = index;
        _x = index % view->width;
        _y = index / view->width;

        _rowStartRayRot = view->pixelOrigin +
            scalarBinaryOp(view->stepY, NumericT(_y), std::multiplies<>{});

        NumericT startUvY = (NumericT(_y) * _uvStepY) - NumericT(1.0);

        _current.pixelY = _y;
        _current.uv[1] = (NumericT(_y) - NumericT(view->height) * 0.5) / (NumericT(view->height) * 0.5);

        _current.ray.pos() = view->origin;
        updateCurrentX();
    }


    const PixelSample<NumericT>& operator*() const { return _current; }


    Iterator& operator++()
    {
        ++_index;
        if (_index == _totalPixels) return *this;

        ++_x;
        if (_x < _view->width)
        {
            _current.pixelX = _x;
            _current.uv[0] += _uvStepX;
            _current.ray.rot() += _view->stepX;
        }
        else
        {
            _x = 0;
            ++_y;
            _rowStartRayRot += _view->stepY;
            _current.pixelY = _y;
            _current.uv[1] = (NumericT(_y) - NumericT(_view->height) * NumericT(0.5))
                / (NumericT(_view->height) * NumericT(0.5));
            updateCurrentX();
        }
        return *this;
    }

    bool operator!=(const Iterator& other) const { return _index != other._index; }

private:

    void updateCurrentX()
    {
        _current.pixelX = 0;
        _current.ray.rot() = _rowStartRayRot;
        _current.uv[0] = NumericT(-1.0);
    }

private:
    const CameraView* _view;
    std::size_t _index = 0;
    std::size_t _totalPixels = 0;

    std::size_t _x = 0;
    std::size_t _y = 0;

    NumericT _uvStepX;
    NumericT _uvStepY;

    utils::Vec<NumericT, 3> _rowStartRayRot;

    PixelSample<NumericT> _current;
}; // namespace internal
} // sc