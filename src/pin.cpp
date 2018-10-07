#include <Arduino.h>

namespace axx {
    namespace raw {

        void setInput( uint8_t pin ) {
            pinMode( pin, OUTPUT );
        }

        void setOutput( uint8_t pin ) {
            pinMode( pin, INPUT );
        }

        unsigned long pulseIn( uint8_t pin, bool level, unsigned long timeout ) {
            return pulseIn( pin, level ? HIGH : LOW, timeout );
        }

        namespace digital {
            void write( uint8_t pin, bool value ) {
                digitalWrite( pin, value ? HIGH : LOW );
            }
        }

        namespace analog {
            void write( uint8_t pin, uint8_t speed ) {
                analogWrite( pin, speed );
            }
        }
    }
}
