#pragma once

#include "pwm.hpp"

// PWM< 23_pin, 0_timer, 0_channel, 16_tbits > pwm23( 100_Hz );

template< gpio_num_t pin1, gpio_num_t pin2, ledc_channel_t channel >
struct Motor : PWM< pin1, 0_timer, channel, 8_tbits >, OutPin< pin2 >
{
	using pwm = PWM< pin1, 0_timer, channel, 8_tbits >;
	using pin = OutPin< pin2 >;

    Motor() : pwm( 100_Hz )
	{
		stop();
    }

    void forward( uint8_t speed = 255 )
    {
		pwm::set_duty( speed );
		pin::set( false );
    }

    void backward( uint8_t speed = 255 )
    {
		pwm::set( true );
        pin::set_duty( -speed - 1 );
    }

    void stop() {
		pwm::set_duty( 0 );
		pin::set( false );
    }
};
