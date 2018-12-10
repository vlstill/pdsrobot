#include <assert.h>
#include "axx.hpp"
#include "pin.hpp"
#include "timing.hpp"
#include "serial.hpp"
#include "motor.hpp"
#include "ultrasonic.hpp"
#include "servo.hpp"
#include "pid.hpp"
#include "ir.hpp"

namespace axx {

using EyeServo = Servo< -1 >;
using LeftMotor = Motor< 7, 6 >;
using RightMotor = Motor< 4, 5 >;

// input capture -- měření délky pulsu
// základ časovače: 16 MHz
// budeme používat fast PWM

void main() {
    EyeServo servo;

    LeftMotor mleft;
    RightMotor mrigh;

    Ir< 3 > ir;
    ir.test();

    serial << "Hello world!\n";

    uint8_t speed = 255;

    mleft.stop();
    mrigh.stop();

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
                    servo.set( -30 );
                    delay( 200_ms );
                    auto dist_l = ultrasonic.distance();

                    servo.set( 0 );
                    delay( 200_ms );
                    auto dist_0 = ultrasonic.distance();

                    servo.set( 30 );
                    delay( 200_ms );
                    auto dist_r = ultrasonic.distance();

                    serial << dist_l << ' ' << dist_0 << ' ' << dist_r << '\n';

                    if ( dist_0 < 50_mm ) {
                        mleft.backward();
                        mrigh.backward();
                        serial << "0\n";
                    } else if ( dist_l < 500_mm ) {
                        mleft.forward();
                        mrigh.backward();
                        serial << "l\n";
                    } else if ( dist_r < 500_mm ) {
                        mleft.backward();
                        mrigh.forward();
                        serial << "r\n";
                    } else {
                        mleft.forward();
                        mrigh.forward();
                        serial << "f\n";
                    }
                }
                break;
        }
    }
}

}
