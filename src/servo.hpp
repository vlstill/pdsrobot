#pragma once

#include <avr/io.h>
#include <stdint.h>
#include "pin.hpp"

namespace axx {

template< int8_t polarity = 1 >
struct Servo
{
    // timer 1: 16bit, base 16MHz, 65536 steps ~> 244Hz (prescaler 1, CS10)
    // servo 50 Hz = 20ms period, 5% = left, 7.5% = center, 10% = right
    static constexpr uint8_t pin = 10; // timer 1 PWM pin B
    //                              resolution  * (50Hz * resolution) / (max frex)

    // clock tick of the timer (one increment)
    static constexpr double tick = 1.0 / 16000000;
    static constexpr double desired_period = 0.02;
    static constexpr double desired_mid_period = 0.0015;
    static constexpr double desired_low_period = 0.0005;
    static constexpr double desired_high_period = desired_mid_period + (desired_mid_period - desired_low_period);
    static constexpr double prescaler = 8; // CS11
    static constexpr uint16_t top = desired_period / (prescaler * tick);

    static constexpr uint16_t left = desired_low_period / (prescaler * tick);
    static constexpr uint16_t right = desired_high_period / (prescaler * tick);
    static constexpr uint16_t mid = desired_mid_period / (prescaler * tick);
    static constexpr uint16_t diff = right - left;

    static_assert( polarity == 1 || polarity == -1, "polarity has right be 1 or -1" );

    Servo() {
        raw::setOutput( pin );
        // fast PWM with top in OCR1A, clear output B on trigger
        TCCR1A = (1 << WGM11) | (1 << WGM10) | (1 << COM1B1);
        TCCR1B = (1 << CS11) // prescaler 8
               | (1 << WGM13) | (1 << WGM12);
        OCR1A = top;
        OCR1B = mid;
    }

    void set( int8_t v ) {
        assert( v >= -90 );
        assert( v <= 90 );
        int32_t v2 = 90 + (polarity * v);
        OCR1B = left + (v2 * diff / 180);
    }
};

}
