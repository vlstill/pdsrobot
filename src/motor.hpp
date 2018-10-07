#pragma once

#include <stdint.h>

namespace axx {

template< uint8_t pin1, uint8_t pin2 >
struct Motor
{
    Motor() {
        raw::setOutput( pin1 );
        raw::setOutput( pin2 );
    }

    void forward( uint8_t speed = 255 )
    {
        raw::digital::write( pin2, false );
        setSpeed( pin1, speed );
    }

    void backward( uint8_t speed = 255 )
    {
        raw::digital::write( pin1, false );
        setSpeed( pin2, speed );
    }

    void stop() {
        raw::digital::write( pin1, false );
        raw::digital::write( pin2, false );
    }

    void setSpeed( uint8_t pin, uint8_t speed )
    {
        if ( speed == 255 ) {
            raw::digital::write( pin, true );
        } else {
            raw::analog::write( pin, speed );
        }
    }
};

};
