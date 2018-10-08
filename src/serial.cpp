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

#undef NUM_OUT

Serial &Serial::operator>>( char &v ) {
    while ( ::Serial.available() == 0 ) { }
    v = ::Serial.read();
    return *this;
}

} // namespace axx
