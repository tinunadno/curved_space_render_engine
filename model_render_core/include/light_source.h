#pragma once
#include "utils/vec.h"

namespace mrc
{

template<typename NumericT>
struct LightSource {
    sc::utils::Vec<NumericT, 3> position;
    sc::utils::Vec<NumericT, 3> direction;
    sc::utils::Vec<float, 3> color;
    NumericT intensity;
    LightSource(
        const sc::utils::Vec<NumericT, 3>& pos,
        const sc::utils::Vec<NumericT, 3>& dir,
        const sc::utils::Vec<NumericT, 3>& col,
        NumericT intensity
    )
        : position{pos}, direction{dir}, color{col}, intensity{intensity}
    { }
    LightSource()
        : position(sc::utils::Vec<NumericT, 3>(0, 0, 0))
        , direction(sc::utils::Vec<NumericT, 3>{1.f, 0, 0})
        , color(sc::utils::Vec<NumericT, 3>{1.f, 1.f, 1.f})
        , intensity(1.f)
    { }
};

} // namespace mrc