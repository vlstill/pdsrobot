#include <Arduino.h>

namespace axx {
    namespace raw {

        using portreg = volatile uint8_t &;

#define PIN_MAP(out_t, to, B, C, D) static out_t pinTo ## to ( uint8_t pin ) { \
            switch ( pin ) { \
                case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7: \
                    return D; \
                case 8: case 9: case 10: case 11: case 12: case 13: \
                    return B; \
                case 14: case 15: case 16: case 17: case 18: case 19: \
                    return C; \
            } \
            return *((uint8_t *)0); \
        } \

        PIN_MAP( portreg, Ddr, DDRB, DDRC, DDRD )
        PIN_MAP( portreg, Port, PORTB, PORTC, PORTD )
        PIN_MAP( portreg, Pin, PINB, PINC, PIND )
        PIN_MAP( uint8_t, Bit, pin - 8, pin - 14, pin )
#undef PIN_MAP

        void setInput( uint8_t pin ) {
            auto &reg = pinToDdr( pin );
            const auto mask = 1 << pinToBit( pin );
            reg &= ~mask;
        }

        void setOutput( uint8_t pin ) {
            auto &reg = pinToDdr( pin );
            const auto mask = 1 << pinToBit( pin );
            reg |= mask;
        }

        unsigned long pulseIn( uint8_t pin, bool level, unsigned long timeout ) {
            return pulseIn( pin, level ? HIGH : LOW, timeout );
        }

        namespace digital {
            void write( uint8_t pin, bool value ) {
                auto &reg = pinToPort( pin );
                const auto mask = 1 << pinToBit( pin );
                if ( value ) {
                    reg |= mask;
                }
                else {
                    reg &= ~mask;
                }
            }

            void flip( uint8_t pin ) {
                pinToPin( pin ) |= 1 << pinToBit( pin );
            }
        }

        namespace analog {
            void write( uint8_t pin, uint8_t speed ) {
                analogWrite( pin, speed );
            }
        }
    }
}
