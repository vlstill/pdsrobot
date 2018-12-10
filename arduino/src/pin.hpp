#pragma once

#include <stdint.h>

namespace axx {
    namespace raw {

        void setOutput( uint8_t pin );
        void setInput( uint8_t pin );

        unsigned long pulseIn( uint8_t pin, bool level, unsigned long timeout = 1000000 );

        namespace digital {
            void write( uint8_t pin, bool value );
        }
        namespace analog {
            void write( uint8_t pin, uint8_t val );
        }
    }

    template< uint8_t pin >
    struct OutPin {
        OutPin() { raw::setOutput( pin ); }

        void write( bool v ) {
            raw::digital::write( pin, v );
        }

        void writeAnalog( uint8_t v ) {
            raw::analog::write( pin, v );
        }
    };
}
