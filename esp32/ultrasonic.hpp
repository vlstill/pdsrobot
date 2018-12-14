#pragma once

#include <chrono>
#include <driver/rmt.h>
#include <soc/rmt_reg.h>
#include "freq.hpp"
#include "util.hpp"

using namespace std::literals;

template< gpio_num_t TRIG_PIN, gpio_num_t ECHO_PIN >
struct Ultrasonic : OutPin< TRIG_PIN >
{
	constexpr static FreqHz TIMER_FREQ = 80_MHz;
	constexpr static std::chrono::nanoseconds TICK { 1000 * 1000 * 1000 / TIMER_FREQ.value };

	constexpr static auto TRIG_CHANNEL = RMT_CHANNEL_0;
	constexpr static auto ECHO_CHANNEL = RMT_CHANNEL_1;

	Ultrasonic()
	{
		std::cout << "tick = " << TICK.count() << std::endl << std::flush;
		trig[0].duration0 = 1 + 10us / TICK;
		trig[0].level0 = 1;
		trig[0].duration1 = 0;
		trig[0].level1 = 0;

		rmt_config_t trig_config;
		trig_config.rmt_mode = RMT_MODE_TX;
		trig_config.channel = TRIG_CHANNEL;
		trig_config.gpio_num = TRIG_PIN;
		trig_config.mem_block_num = trig.size();
		trig_config.clk_div = 1;
		trig_config.tx_config.loop_en = false;
		trig_config.tx_config.carrier_en = false; // no modulation
		trig_config.tx_config.idle_output_en = true; // when idle…
		trig_config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW; // send 0

		rmt_config( &trig_config ) || Die( "Ultrasonic: trigger RMT failed to config" );
		rmt_driver_install( trig_config.channel, 0, 0 )
			|| Die( "Ultrasonic: trigger RMT failed in driver install" );

		rmt_config_t echo_config;
		echo_config.rmt_mode = RMT_MODE_RX;
		echo_config.channel = ECHO_CHANNEL;
		echo_config.gpio_num = ECHO_PIN;
		echo_config.mem_block_num = 2;
		echo_config.clk_div = 1;
		echo_config.rx_config.filter_en = true;
		echo_config.rx_config.filter_ticks_thresh = 1;
		echo_config.rx_config.idle_threshold = 512;
		rmt_config( &echo_config ) || Die( "Ultrasonic: echo RMT failed in config" );
		rmt_driver_install( echo_config.channel, 1024, 0 )
			|| Die( "Ultrasonic: echo RMT failed in driver install" );
		rmt_get_ringbuf_handle( echo_config.channel, &echo_rb )
			|| Die( "Ultrasonic: echo RMT ringbuf get failed" );

		std::cout << "idle_threshold = " << echo_config.rx_config.idle_threshold << std::endl << std::flush;
	}

	int measure() {
		rmt_rx_start( ECHO_CHANNEL, true ) || Die( "Ultrasonic: echo receiving start failed" );
		// rmt_write_items( TRIG_CHANNEL, &trig[0], trig.size(), false /* do not wait */ )
		//	|| Die( "Ultrasonic: trigger failed" );
		PWM< TRIG_PIN, 2_timer, 2_channel, 2_tbits > pwm( 300_kHz );
		pwm.set_duty_perc( 50 );

		size_t rcv_size = 0;
		rmt_item32_t *echo = nullptr;
		do {
			echo = reinterpret_cast< rmt_item32_t * >( xRingbufferReceive( echo_rb, &rcv_size, 1000 ) );
			std::cout << "wait " << rcv_size << std::endl;
			if ( echo )
				vRingbufferReturnItem( echo_rb, reinterpret_cast< void * >( echo ) );
		} while ( echo == nullptr );
		rmt_rx_stop( ECHO_CHANNEL );
		std::cout << "duration0 " << echo->duration0
				  << " level0 " << echo->level0
				  << " duration1 " << echo->duration1
				  << " level1 " << echo->level1
				  << std::endl;
		return 0;
	}

    std::array< rmt_item32_t, 1 > trig{};
	RingbufHandle_t echo_rb = nullptr;
};
