#pragma once

#include <stdint.h>
#include <avr/io.h>

namespace axx {

// motors use timer0: OC0A (pin 6), OC0B (pin 5)
template< uint8_t pin1, uint8_t pin2 >
struct Motor
{
    static_assert( (pin1 == 6 && pin2 != 5) || (pin1 == 5 && pin2 != 6)
                   || (pin1 != 6 && pin2 == 5) || (pin1 != 5 && pin2 == 6 ),
                   "exactly one motor pin has to be connected to timer0/PWM0 pins 5 or 6" );

    constexpr static uint8_t pwmPin = pin1 == 5 || pin1 == 6 ? pin1 : pin2;
    constexpr static uint8_t basePin = pin1 == 5 || pin1 == 6 ? pin2 : pin1;
    constexpr static decltype( OCR0A ) trigger() {
        return pin1 == 6 || pin2 == 6 ? OCR0A : OCR0B;
    }

    constexpr static uint8_t norm_speed( uint8_t s ) {
        return pwmPin == pin1 ? s : -s - 1;
    }

    constexpr static bool F = pwmPin != pin1;
    constexpr static bool B = !F;

    Motor() {
        raw::setOutput( pwmPin );
        raw::setOutput( basePin );

        // setup PWM at ~30Hz ~> divisor 1024 + phase correct for 16MHz clock on UNO

        // phase correct PWM + both outputs
        TCCR0A = (1 << WGM00) | (1 << COM0B1) | (1 << COM0A1);
        TCCR0B = (1 << CS02) | (1 << CS00); // 1 / 1024

        stop();
    }

    void forward( uint8_t speed = 255 )
    {
        raw::digital::write( basePin, F );
        trigger() = norm_speed( speed );
    }

    void backward( uint8_t speed = 255 )
    {
        raw::digital::write( basePin, B );
        trigger() = -norm_speed( speed ) - 1;
    }

    void stop() {
        raw::digital::write( basePin, false );
        trigger() = 0;
    }
};

};
