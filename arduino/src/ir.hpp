#pragma once

#include <Adafruit_MCP3008.h>
#include "serial.hpp"
#include "timing.hpp"

namespace axx {

template< uint8_t Cs >
struct Ir {

    Ir() {
        adc.begin( Cs );
    }

    void test() {
        while ( true ) {
            for (int chan=0; chan < 8; chan++) {
                serial << adc.readADC(chan) << (chan != 7 ? "," : "\n");
            }
            delay( 10_ms );
        }
    }

  private:
    Adafruit_MCP3008 adc;
};

};
