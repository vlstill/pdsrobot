#include "timing.hpp"
#include <util/delay.h>

namespace axx {

void delay( seconds s ) {
    _delay_ms( duration_cast< milliseconds >( s ).count() );
}

void delay( milliseconds ms ) {
    _delay_ms( ms.count() );
}

void delay( microseconds us ) {
    _delay_us( us.count() );
}

}
