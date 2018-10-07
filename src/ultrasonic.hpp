#pragma once

#include <stdint.h>

namespace axx {

template< uint8_t trigPin, uint8_t echoPin >
struct Ultrasonic {
    Ultrasonic() {
        raw::setOutput( trigPin );
        raw::setInput( echoPin );
    }

    // in millimeters
    unsigned long distance() {
        // (high level time * sound velocity = 340m/s) / 2
        raw::digital::write( trigPin, true );
        delay( 10_us );
        raw::digital::write( trigPin, false );
        // pulse is in microseconds
        constexpr unsigned long max = 4000;
        auto dist = pulseToMM( raw::pulseIn( echoPin, true, mmToPulse( max ) ) );
        return dist == 0 ? -1 : dist;
    }

    static constexpr unsigned long pulseToMM( unsigned long pulse ) {
        return (pulse * 340) / 2 / 1000;
    }

    static constexpr unsigned long mmToPulse( unsigned long mm ) {
        return (mm * 1000 * 2) / 340;
    }

    static constexpr unsigned long err = -1;
};

}
