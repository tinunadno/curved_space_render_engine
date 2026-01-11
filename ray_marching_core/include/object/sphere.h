#pragma once

#include "iobject.h"

namespace rmc::object
{

template<typename NumericT>
class Sphere: public IObject<NumericT>
{
public:
    Sphere(const sc::utils::Vec<NumericT, 3>& position,
        NumericT radius,
        const sc::utils::Vec<NumericT, 3>& scale = sc::utils::Vec<NumericT, 3>{1., 1., 1.})
        : _position(position)
        , _radius(radius)
        , _scale(scale)
    { }

    NumericT sdf(const sc::utils::Vec<NumericT, 3>& p) const final
    {
        auto q = p - _position;
        sc::utils::Vec<NumericT, 3> r{
            _scale[0] * _radius,
            _scale[1] * _radius,
            _scale[2] * _radius
        };

        auto p_div_r = q;
        p_div_r[0] /= r[0];
        p_div_r[1] /= r[1];
        p_div_r[2] /= r[2];

        NumericT k0 = sc::utils::len(p_div_r);

        auto p_div_r2 = q;
        p_div_r2[0] /= (r[0] * r[0]);
        p_div_r2[1] /= (r[1] * r[1]);
        p_div_r2[2] /= (r[2] * r[2]);

        NumericT k1 = sc::utils::len(p_div_r2);

        return k0 * (k0 - 1.0) / k1;
    }

    sc::utils::Vec<NumericT, 3> normal(const sc::utils::Vec<NumericT, 3>& vec) const final
    {
        auto q = (vec - _position) / (_scale * _scale);
        return sc::utils::norm(q);
    }

private:
    sc::utils::Vec<NumericT, 3> _position;
    NumericT _radius;
    sc::utils::Vec<NumericT, 3> _scale;
    shader::Material _mat;
};

} // rmc::object