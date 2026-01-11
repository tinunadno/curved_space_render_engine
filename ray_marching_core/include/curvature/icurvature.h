#pragma once

namespace rmc::curvature
{

template<typename NumericT>
class ICurvature
{
public:
    virtual ~ICurvature() = default;
    virtual void deflect(const sc::utils::Vec<NumericT,3>& pos, const sc::utils::Vec<NumericT,3>& dir);
};

} // namespace rmc::curvature