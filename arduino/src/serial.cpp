#include <Arduino.h>
#include "serial.hpp"

namespace axx {

Serial serial;

Serial &Serial::operator<<( const char *v ) {
    ::Serial.print( v );
    return *this;
}

#define NUM_OUT( T ) Serial &Serial::operator<<( T v ) { \
    ::Serial.print( v ); \
    return *this; \
}

NUM_OUT( char )

NUM_OUT( short )
NUM_OUT( int )
NUM_OUT( long )

NUM_OUT( unsigned short )
NUM_OUT( unsigned )
NUM_OUT( unsigned long )

namespace detail {

Serial &dump_long_long( Serial &s, bool negative, unsigned long long ull ) {
    char buf[20] = { 0 };
    char *ptr = &buf[ sizeof( buf ) - 2 ];
    *ptr = '0';
    for ( ; ull; --ptr, ull /= 10 )
        *ptr = '0' + (ull % 10);
    if ( negative ) {
        *ptr = '-';
        --ptr;
    }
    return serial << ptr + 1;
}

}

Serial &Serial::operator<<( long long v ) {
    return detail::dump_long_long( *this, v < 0, v < 0 ? -v : v );
}

Serial &Serial::operator<<( unsigned long long v ) {
    return detail::dump_long_long( *this, false, v );
}

#undef NUM_OUT

Serial &Serial::operator>>( char &v ) {
    while ( ::Serial.available() == 0 ) { }
    v = ::Serial.read();
    return *this;
}

} // namespace axx
