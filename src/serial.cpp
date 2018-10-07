#include <Arduino.h>
#include "serial.hpp"

namespace axx {

namespace detail
{
    struct Init {
        Init() { ::Serial.begin( 9600 ); }
    };

    Init init;
}

Serial serial;

Serial &Serial::operator<<( const char *v ) {
    ::Serial.print( v );
    return *this;
}

Serial &Serial::operator<<( char v ) {
    ::Serial.print( v );
    return *this;
}

Serial &Serial::operator>>( char &v ) {
    v = ::Serial.read();
    return *this;
}

} // namespace axx
