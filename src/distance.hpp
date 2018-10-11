#pragma once

#include "scaled.hpp"

namespace axx {

template< typename Repr, typename Period = axx::ratio< 1 > >
struct distance : detail::Scaled< duration< Repr, Period >, Repr >
{
    using rep = Repr;
    using period = Period;
    using Base = detail::Scaled< duration< Repr, Period >, rep >;

    using Base::Scaled;

    constexpr Repr count() { return Base::_val; }
};

using meters = distance< long long >;
using millimeters = distance< long long, ratio< 1, 1000 > >;

SCALED_OPS(meters, m)
SCALED_OPS(millimeters, mm)

}
