#include <assert.h>
#include "axx.hpp"
#include "pin.hpp"
#include "timing.hpp"
#include "serial.hpp"
#include "motor.hpp"
#include "ultrasonic.hpp"
#include "servo.hpp"

namespace axx {

using EyeServo = Servo< 8, -1 >;
using EyeUltrasonic = Ultrasonic< 12, 13 >;
using LeftMotor = Motor< 2, 3 >;
using RightMotor = Motor< 4, 5 >;

void main() {
    EyeServo servo;

    LeftMotor mleft;
    RightMotor mrigh;

    EyeUltrasonic ultrasonic;

    serial << "Hello world!\n";

    while ( true ) {
        char c;
        serial >> c;
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
                    serial << "(error)\n";
                else
                    serial << dist << "\n";
                break;
                }
            case 'g':
            case 'G':
                mleft.forward();
                mrigh.forward();
                while ( true ) {
                    auto dist = ultrasonic.distance();
                    serial << dist << "\n";
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
