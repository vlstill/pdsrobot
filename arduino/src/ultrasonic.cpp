#include "ultrasonic.hpp"

namespace axx {

Ultrasonic ultrasonic;


ISR(TIMER1_CAPT_vect) {
    if ( !ultrasonic.up ) {
        ultrasonic.setTrigEdge( false );
        ultrasonic.val_up = ICR1;
        ultrasonic.cnt_up = ultrasonic.cnt;
        ultrasonic.up = true;
    } else {
        ultrasonic.val_down = ICR1;
        ultrasonic.cnt_down = ultrasonic.cnt;
        ultrasonic.done = true;
    }
}

ISR(TIMER1_OVF_vect) {
    ++ultrasonic.cnt;
}

}
