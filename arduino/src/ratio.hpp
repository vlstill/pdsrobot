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

}
