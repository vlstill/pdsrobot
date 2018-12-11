#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include <chrono>
#include <thread>
#include "pwm.hpp"

// motor1: 23, 22

using namespace std::literals;

extern "C" void app_main()
{
	PWM< 23_pin, 0_timer, 0_channel, 16_tbits > pwm23( 100_Hz );
	PWM< 19_pin, 1_timer, 1_channel, 16_tbits > pwm19( 100_Hz );
	int perc = 0;
    while( 1 ) {
		perc += 10;
		if ( perc > 100 )
			perc = 0;
		pwm23.set_duty_perc( perc );
		pwm19.set_duty_perc( perc );
		std::this_thread::sleep_for( 1s );
    }
}
