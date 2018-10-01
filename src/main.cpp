#include <Arduino.h>
#include <Servo.h>
#include <stdint.h>
#include <assert.h>

namespace pds {

template< uint8_t pin1, uint8_t pin2 >
struct Motor
{
    void forward( uint8_t speed = 255 )
    {
        digitalWrite( pin2, LOW );
        setSpeed( pin1, speed );
    }

    void backward( uint8_t speed = 255 )
    {
        digitalWrite( pin1, LOW );
        setSpeed( pin2, speed );
    }

    void stop() {
        setSpeed( pin1, LOW );
        setSpeed( pin2, LOW );
    }

    void setSpeed( uint8_t pin, uint8_t speed )
    {
        if ( speed == 255 ) {
            digitalWrite( pin, HIGH );
        } else {
            analogWrite( pin, speed );
        }
    }
};

template< uint8_t pin, int8_t polarity = 1 >
struct Servo
{
    static_assert( polarity == 1 || polarity == -1, "polarity has to be 1 or -1" );

    Servo() {
        _s.attach( pin );
    }

    void set( int8_t v ) {
        assert( v >= -90 );
        assert( v <= 90 );
        _s.write( 90 + (polarity * v) );
    }
  private:
    ::Servo _s;
};

}

void setup() {
    pds::Servo< 8, -1 > servo;
    Serial.begin( 9600 );

    pds::Motor< 2, 3 > mleft;
    pds::Motor< 4, 5 > mrigh;

    for ( uint8_t i = 2; i < 6; ++i )
        pinMode( i, OUTPUT );

    Serial.println( "Hello world!" );

    while ( true ) {
        char c = Serial.read();
        switch ( c ) {
            case 'w': // forward
            case 'W':
                mleft.forward();
                mrigh.forward();
                break;
            case 'a': // left
            case 'A':
                mleft.backward();
                mrigh.forward();
                break;
            case 's': // back
            case 'S':
                mleft.backward();
                mrigh.backward();
                break;
            case 'd': // right
            case 'D':
                mleft.forward();
                mrigh.backward();
                break;
            case 'q':
            case 'Q':
                servo.set( -90 );
                break;
            case 'e':
            case 'E':
                servo.set( 90 );
                break;
            case 'z':
            case 'Z':
                servo.set( 0 );
            case 'x':
            case 'X':
                mleft.stop();
                mrigh.stop();
        }
    }
}

void loop() { }
