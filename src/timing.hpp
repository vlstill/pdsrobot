#pragma once

#include "scaled.hpp"

namespace axx {

template< typename Repr, typename Period = axx::ratio< 1 > >
struct duration : Scaled< Repr, Period >
{
    using Base = Scaled< Repr, Period >;
    using Base::Scaled;

    constexpr Repr count() { return Base::_val; }
};

template< typename DurationTo, typename ReprFrom, typename PeriodFrom >
constexpr DurationTo duration_cast( const duration< ReprFrom, PeriodFrom > &d ) {
    return scale_cast< DurationTo >( d );
}

using seconds = duration< long long >;
using milliseconds = duration< long long, ratio< 1, 1000 > >;
using microseconds = duration< long long, ratio< 1, 1000ll * 1000ll > >;
using nanoseconds = duration< long long, ratio< 1, 1000ll * 1000ll * 1000ll > >;

SCALED_OPS(seconds, s)
SCALED_OPS( milliseconds, ms)
SCALED_OPS( microseconds, us)
SCALED_OPS( nanoseconds, ns )

void delay( seconds );
void delay( milliseconds );
void delay( microseconds );

} // namespace axx
