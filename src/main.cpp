#include <Servo.h>
#include <stdint.h>
#include <assert.h>
#include "axx.hpp"

namespace axx {

template< uint8_t pin1, uint8_t pin2 >
struct Motor
{
    Motor() {
        pinMode( pin1, OUTPUT );
        pinMode( pin2, OUTPUT );
    }

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

template< uint8_t trigPin, uint8_t echoPin >
struct Ultrasonic {
    Ultrasonic() {
        pinMode( trigPin, OUTPUT );
        pinMode( echoPin, INPUT );
    }

    // in millimeters
    unsigned long distance() {
        // (high level time * sound velocity = 340m/s) / 2
        digitalWrite( trigPin, HIGH );
        delayMicroseconds( 10 );
        digitalWrite( trigPin, LOW );
        // pulse is in microseconds
        constexpr unsigned long max = 4000;
        auto dist = pulseToMM( pulseIn( echoPin, HIGH, mmToPulse( max ) ) );
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

using EyeServo = Servo< 8, -1 >;
using EyeUltrasonic = Ultrasonic< 12, 13 >;
using LeftMotor = Motor< 2, 3 >;
using RightMotor = Motor< 4, 5 >;

void main() {
    EyeServo servo;
    Serial.begin( 9600 );

    LeftMotor mleft;
    RightMotor mrigh;

    EyeUltrasonic ultrasonic;

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
                break;
            case 'x':
            case 'X':
                mleft.stop();
                mrigh.stop();
                break;
            case 'm':
            case 'M':
                {
                auto dist = ultrasonic.distance();
                if ( dist == ultrasonic.err )
                    Serial.println( "(error)" );
                else
                    Serial.println( dist );
                break;
                }
            case 'g':
            case 'G':
                mleft.forward();
                mrigh.forward();
                while ( true ) {
                    auto dist = ultrasonic.distance();
                    Serial.println( dist );
                    if ( dist == ultrasonic.err )
                        continue;
                    if ( dist < 50 ) {
                        mleft.stop();
                        mrigh.stop();
                        break;
                    }
                }
                break;
        }
    }
}

}
