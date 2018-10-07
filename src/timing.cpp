#include <Arduino.h>
#include "timing.hpp"

namespace axx {

void delay( seconds s ) {
    ::delay( s.count() * 1000 );
}

void delay( milliseconds ms ) {
    ::delay( ms.count() );
}

void delay( microseconds us ) {
    ::delayMicroseconds( us.count() );
}

}
