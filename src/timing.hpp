#pragma once

namespace axx {

template< long long Num, long long Denom = 1 >
struct ratio {
    static constexpr long long num() { return Num; }
    static constexpr long long den() { return Denom; }
};

template< typename T >
constexpr T gcd( T a, T b ) {
    return b == 0 ? a : gcd( b, a % b );
}

template< typename, typename > struct Div;

template< long long NA, long long DA, long long NB, long long DB >
struct Div< ratio< NA, DA >, ratio< NB, DB > >  {
    static constexpr long long RNum = NA * DB;
    static constexpr long long RDen = DA * NB;
    using type = ratio< RNum / gcd( RNum, RDen ), RDen / gcd( RNum, RDen ) >;
};

template< typename Repr, typename Period = axx::ratio< 1 > >
struct duration
{
    using rep = Repr;
    using period = Period;

    constexpr duration() = default;
    constexpr explicit duration( const rep &r ) : _val( r ) { }

    constexpr Repr count() { return _val; }

  private:
    Repr _val;
};

template< typename Repr, typename Period >
constexpr duration< Repr, Period > operator-( duration< Repr, Period > a, duration< Repr, Period > b ) {
    return duration< Repr, Period >{ a.count() - b.count() };
}

template< typename DurationTo, typename ReprFrom, typename PeriodFrom >
constexpr DurationTo duration_cast( const duration< ReprFrom, PeriodFrom > &d ) {
    using Conv = typename Div< PeriodFrom, typename DurationTo::period >::type;
    return DurationTo{ d.count() * Conv::num() / Conv::den() };
}

using seconds = duration< long long >;
using milliseconds = duration< long long, ratio< 1, 1000 > >;
using microseconds = duration< long long, ratio< 1, 1000ll * 1000ll > >;
using nanoseconds = duration< long long, ratio< 1, 1000ll * 1000ll * 1000ll > >;

constexpr seconds operator "" _s( unsigned long long s ) {
    return seconds( s );
}

constexpr milliseconds operator "" _ms( unsigned long long ms ) {
    return milliseconds( ms );
}

constexpr microseconds operator "" _us( unsigned long long us ) {
    return microseconds( us );
}

constexpr nanoseconds operator "" _ns( unsigned long long ns ) {
    return nanoseconds( ns );
}

void delay( seconds );
void delay( milliseconds );
void delay( microseconds );

} // namespace axx
