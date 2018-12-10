#pragma once

#include "type_traits.hpp"
#include "ratio.hpp"
#include "serial.hpp"

namespace axx {
namespace detail { struct ScaledTag { }; }

template< typename Rep, typename Period >
struct Scaled : detail::ScaledTag
{
    using rep = Rep;
    using period = Period;

    constexpr Scaled() = default;
    constexpr explicit Scaled( const Rep &r ) : _val( r ) { }

    Scaled &scaled() { return *this; }
    const Scaled &scaled() const { return *this; }

    Rep value() { return _val; }

    Rep _val;
};

template< typename ScaledTo, typename ScaledFrom >
constexpr ScaledTo scale_cast( const ScaledFrom &s ) {
    using Conv = typename Div< typename ScaledFrom::period, typename ScaledTo::period >::type;
    return ScaledTo{ s._val * Conv::num() / Conv::den() };
}

namespace detail {

#define SCALED_REL(rel) template< typename S > \
constexpr enable_if_t< is_base_of< ScaledTag, S >::value, bool > \
    operator rel( const S &a, const S &b ) \
{ \
    return a._val rel b._val; \
}

SCALED_REL(==)
SCALED_REL(!=)
SCALED_REL(<)
SCALED_REL(<=)
SCALED_REL(>)
SCALED_REL(>=)

#undef SCALED_REL

template< typename S >
constexpr enable_if_t< is_base_of< ScaledTag, S >::value, S >
    operator-( const S &a, const S &b )
{
    return S{ a._val - b._val };
}

template< typename S >
constexpr enable_if_t< is_base_of< ScaledTag, S >::value, S & >
    operator-=( S &a, const S &b )
{
    a._val -= b._val;
    return a;
}

template< typename S >
constexpr enable_if_t< is_base_of< ScaledTag, S >::value, S >
    operator+( const S &a, const S &b )
{
    return S{ a._val + b._val };
}

template< typename S >
constexpr enable_if_t< is_base_of< ScaledTag, S >::value, S & >
    operator+=( S &a, const S &b )
{
    a._val += b._val;
    return a;
}

#define SCALED_CON_OP(op) \
template< typename S > \
constexpr enable_if_t< is_base_of< ScaledTag, S >::value, S > \
    operator op( const S &a, const typename S::rep &b ) \
{ \
    return S{ a._val op b }; \
} \
\
template< typename S > \
constexpr enable_if_t< is_base_of< ScaledTag, S >::value, S > \
    operator op( const typename S::rep &a, const S &b ) \
{ \
    return S{ a op b._val }; \
} \
\
template< typename S > \
constexpr enable_if_t< is_base_of< ScaledTag, S >::value, S & > \
    operator op ## =( S &a, const typename S::rep &b ) \
{ \
    a._val op ## = b; \
    return a;\
}

SCALED_CON_OP(+)
SCALED_CON_OP(-)
SCALED_CON_OP(*)
SCALED_CON_OP(/)

#undef SCALED_CON_OP

#define SCALED_OPS(type, lit) constexpr inline type operator "" _ ## lit( unsigned long long lit ) { \
    return type( lit ); \
} \
\
inline Serial &operator<<( Serial &serial, type &lit ) {\
    return serial << lit._val << " " #lit; \
}


}}
