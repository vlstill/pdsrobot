#pragma once

#include <Servo.h>
#include <stdint.h>

namespace axx {

template< uint8_t pin, int8_t polarity = 1 >
struct Servo
{
    static_assert( polarity == 1 || polarity == -1, "polarity has to be 1 or -1" );

    Servo() {
        _s.attach( pin );
    }

    void set( int8_t v ) {
        assert( v >= -90 );
        assert( v <= 90 );
        _s.write( 90 + (polarity * v) );
    }
  private:
    ::Servo _s;
};

}
