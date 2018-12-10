#pragma once

namespace axx {

template< bool, typename = void >
struct enable_if { };

template< typename T >
struct enable_if< true, T > {
    using type = T;
};

template< bool v, typename T = void >
using enable_if_t = typename enable_if< v, T >::type;

template< typename Base, typename Derived >
struct is_base_of {
    static constexpr bool value = __is_base_of( Base, Derived );
};

}
