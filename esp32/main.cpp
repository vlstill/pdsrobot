#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include <chrono>
#include <thread>
#include "motor.hpp"
#include "ultrasonic.hpp"

// motor1: 23, 22

using namespace std::literals;

extern "C" void app_main()
{
	Motor< 23_pin, 22_pin, 0_channel > m1;
	Motor< 19_pin, 21_pin, 1_channel > m2;
	Ultrasonic< 18_pin, 5_pin > ultrasonic;
	uint8_t speed = 0;
    while( 1 ) {
		speed += 16;
//		m1.forward( speed );
//		m2.forward( speed );
		std::this_thread::sleep_for( 1s );
		std::cout << "measure " << ultrasonic.measure() << std::endl;
    }
}
