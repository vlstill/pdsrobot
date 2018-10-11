#pragma once

#include "pin.hpp"
#include "timing.hpp"
#include "distance.hpp"
#include "serial.hpp"
#include "servo.hpp"
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

namespace axx {

struct Ultrasonic {
    static constexpr uint8_t trigPin = 9;
    static constexpr uint8_t echoPin = 8; // ICP1

    Ultrasonic() {
        raw::setOutput( trigPin );
        raw::setInput( echoPin );

        sei(); // enable all interrupts
    }

    static void setTrigEdge( bool val ) {
        if ( val )
            TCCR1B |= (1 << ICES1);
        else
            TCCR1B &= ~(1 << ICES1);
    }

    // see interrupt def in CPP

    struct WithInterrupt {
        WithInterrupt() { TIMSK1 = (1 << ICIE1) | (1 << TOIE1); sei(); }
        ~WithInterrupt() { TIMSK1 = 0; }
    };

    millimeters distance() {
        setTrigEdge( true );

        {
            WithInterrupt _;
            cnt = 0;
            done = up = false;
            raw::digital::write( trigPin, true );
            delay( 10_ms );
            raw::digital::write( trigPin, false );
            while ( !done ) { }
        }

        nanoseconds time = pulseTime( cnt_down, val_down ) - pulseTime( cnt_up, val_up );
        auto dist = pulseToMM( time );
        return dist;
    }

    static constexpr nanoseconds pulseTime( uint8_t cnt, uint16_t val ) {
        return Servo<>::period * cnt + val * Servo<>::tick;
    }

    static constexpr millimeters pulseToMM( nanoseconds pulse ) {
        return scale_cast< millimeters >( (pulse * 340) / 2 );
    }

    volatile uint8_t cnt, cnt_up, cnt_down;
    volatile uint16_t val_up = 0, val_down = 0;
    volatile bool done = false;
    volatile bool up = false;
};

extern Ultrasonic ultrasonic;

}
