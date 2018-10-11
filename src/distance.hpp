#pragma once

#include "scaled.hpp"

namespace axx {

template< typename Repr, typename Period = axx::ratio< 1 > >
struct distance : Scaled< Repr, Period >
{
    using Base = Scaled< Repr, Period >;
    using Base::Scaled;
};

using meters = distance< long long >;
using centimeters = distance< long long, ratio< 1, 100 > >;
using millimeters = distance< long long, ratio< 1, 1000 > >;

SCALED_OPS(meters, m)
SCALED_OPS(centimeters, cm)
SCALED_OPS(millimeters, mm)

template< typename DistanceTo, typename ReprFrom, typename PeriodFrom >
constexpr DistanceTo distance_cast( const distance< ReprFrom, PeriodFrom > &d ) {
    return scale_cast< DistanceTo >( d );
}

}
