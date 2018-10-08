#include <Arduino.h>
#include "serial.hpp"

namespace axx {

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
    while ( ::Serial.available() == 0 ) { }
    v = ::Serial.read();
    return *this;
}

} // namespace axx
