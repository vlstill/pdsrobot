#pragma once

namespace axx {

template< long long Num, long long Denom = 1 >
struct ratio {
    static constexpr long long num() { return Num; }
    static constexpr long long den() { return Denom; }
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

using seconds = duration< long long >;
using milliseconds = duration< long long, ratio< 1, 1000 > >;
using microseconds = duration< long long, ratio< 1, 1000ll * 1000ll > >;

constexpr seconds operator "" _s( unsigned long long s ) {
    return seconds( s );
}

constexpr milliseconds operator "" _ms( unsigned long long ms ) {
    return milliseconds( ms );
}

constexpr microseconds operator "" _us( unsigned long long us ) {
    return microseconds( us );
}

void delay( seconds );
void delay( milliseconds );
void delay( microseconds );

} // namespace axx
