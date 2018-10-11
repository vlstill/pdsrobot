#pragma once

#include <avr/io.h>
#include <stdint.h>
#include "pin.hpp"
#include "timing.hpp"
#include <assert.h>

namespace axx {

template< int8_t polarity = 1 >
struct Servo
{
    // timer 1: 16bit, base 16MHz, 65536 steps ~> 244Hz (prescaler 1, CS10)
    // servo 50 Hz = 20ms period, 5% = TIMER_LEFT, 7.5% = center, 10% = TIMER_RIGHT
    static constexpr uint8_t pin = 10; // timer 1 PWM pin B
    //                              resolution  * (50Hz * resolution) / (max frex)

    // clock tick of the timer (one increment)
    static constexpr double PRESCALER = 8; // CS11
    static constexpr double TICK = PRESCALER / 16000000;
    static constexpr double DESIRED_PERIOD = 0.02;
    static constexpr double DESIRED_MID_PERIOD = 0.0015;
    static constexpr double DESIRED_LEFT_PERIOD = 0.0005;
    static constexpr double DESIRED_RIGHT_PERIOD = DESIRED_MID_PERIOD + (DESIRED_MID_PERIOD - DESIRED_LEFT_PERIOD);
    static constexpr uint16_t TIMER_TOP = DESIRED_PERIOD / TICK;

    static constexpr uint16_t TIMER_LEFT = DESIRED_LEFT_PERIOD / TICK;
    static constexpr uint16_t TIMER_RIGHT = DESIRED_RIGHT_PERIOD / TICK;
    static constexpr uint16_t TIMER_MID = DESIRED_MID_PERIOD / TICK;
    static constexpr uint16_t TIMER_STEPS = TIMER_RIGHT - TIMER_LEFT;

    static constexpr nanoseconds period{ uint64_t( 1000ll * 1000ll * 1000ll * TICK * TIMER_TOP ) };
    static constexpr nanoseconds tick{ uint64_t( 1000ll * 1000ll * 1000ll * TICK ) };

    static_assert( polarity == 1 || polarity == -1, "polarity has TIMER_RIGHT be 1 or -1" );

    Servo() {
        raw::setOutput( pin );
        // fast PWM with TIMER_TOP in OCR1A, clear output B on trigger
        TCCR1A = (1 << WGM11) | (1 << WGM10) | (1 << COM1B1);
        TCCR1B = (1 << CS11) // prescaler 8
               | (1 << WGM13) | (1 << WGM12);
        OCR1A = TIMER_TOP;
        OCR1B = TIMER_MID;
    }

    void set( int8_t v ) {
        assert( v >= -90 );
        assert( v <= 90 );
        int32_t v2 = 90 + (polarity * v);
        OCR1B = TIMER_LEFT + (v2 * TIMER_STEPS / 180);
    }
};

}
