#include <assert.h>
#include "axx.hpp"
#include "pin.hpp"
#include "timing.hpp"
#include "serial.hpp"
#include "motor.hpp"
#include "ultrasonic.hpp"
#include "servo.hpp"

namespace axx {

using EyeServo = Servo< -1 >;
using EyeUltrasonic = Ultrasonic< 12, 13 >;
using LeftMotor = Motor< 7, 6 >;
using RightMotor = Motor< 4, 5 >;

void main() {
    EyeServo servo;

    LeftMotor mleft;
    RightMotor mrigh;

    EyeUltrasonic ultrasonic;

    serial << "Hello world!\n";

    uint8_t speed = 255;

    while ( true ) {
        char c;
        serial >> c;
        serial << c << "\n";
        switch ( c ) {
            case 'w': // forward
            case 'W':
                mleft.forward( speed );
                mrigh.forward( speed );
                break;
            case 'a': // left
            case 'A':
                mleft.backward( speed );
                mrigh.forward( speed );
                break;
            case 's': // back
            case 'S':
                mleft.backward( speed );
                mrigh.backward( speed );
                break;
            case 'd': // right
            case 'D':
                mleft.forward( speed );
                mrigh.backward( speed );
                break;
            case 'q':
            case 'Q':
                servo.set( -90 );
                break;
            case 'e':
            case 'E':
                servo.set( 90 );
                break;
            case 'r':
            case 'R':
                servo.set( -45 );
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
            case 'o':
            case 'O':
                --speed;
                serial << "speed: " << int(speed) << "\n";
                mleft.forward( speed );
                mrigh.forward( speed );
                break;
            case 'p':
            case 'P':
                ++speed;
                serial << "speed: " << int(speed) << "\n";
                mleft.forward( speed );
                mrigh.forward( speed );
                break;
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
