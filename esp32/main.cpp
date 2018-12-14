#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "soc/soc.h"
#include "soc/mcpwm_struct.h"
#include "soc/mcpwm_reg.h"
#include "sdkconfig.h"
#include <chrono>
#include <thread>
#include "motor.hpp"
#include "ultrasonic.hpp"
#include <atomic>

static_assert( ATOMIC_INT_LOCK_FREE );

constexpr FreqHz BASE_CLOCK = FreqHz( APB_CLK_FREQ ); // 80 MHz
constexpr static std::chrono::nanoseconds BASE_TICK { 1000 * 1000 * 1000 / BASE_CLOCK.value };

constexpr gpio_num_t MOTOR_PINS[2][2] = { { 23_pin, 22_pin }, { 19_pin, 21_pin } };
constexpr auto MOTOR_MCPWM = MCPWM_UNIT_0;
constexpr FreqHz MOTOR_MWM_FREQ = 50_Hz;

constexpr gpio_num_t ULTRASONIC_TRIG = 18_pin;
constexpr static auto ULTRASONIC_TRIG_CHANNEL = RMT_CHANNEL_0;
constexpr gpio_num_t ULTRASONIC_ECHO = 5_pin;
constexpr auto ULTRASONIC_CAPTURE = MCPWM_CAP_0;
constexpr auto CAP0_INT_EN = BIT(27);

// motor1: 23, 22

using namespace std::literals;

template< auto N >
void motor_set( OutPin< N > &base, mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, mcpwm_operator_t op_num, float speed ) {
    if ( speed < 0 ) {
        base.set( true );
        speed = 100 - speed;
    } else {
        base.set( false );
    }
    mcpwm_set_duty( mcpwm_num, timer_num, op_num, speed ) || Die( "Motor PWM set failed" );
}

struct CaptureVal {
    std::atomic< uint32_t > start{ 0 };
    std::atomic< uint32_t > end{ 0 };
    std::atomic< bool > done{ false };
};

void IRAM_ATTR capture_handler( void *arg ) {
    CaptureVal &val = *reinterpret_cast< CaptureVal * >( arg );
    auto status = MCPWM0.int_st.val;
    if ( status & CAP0_INT_EN ) {
        auto edge = mcpwm_capture_signal_get_edge( MCPWM_UNIT_0, MCPWM_SELECT_CAP0 );
        if ( edge != 1 ) { // down
            MCPWM0.int_ena.val &= ~CAP0_INT_EN;
            val.end = mcpwm_capture_signal_get_value( MCPWM_UNIT_0, MCPWM_SELECT_CAP0 );
            val.done = true;
        } else {
            val.start = mcpwm_capture_signal_get_value( MCPWM_UNIT_0, MCPWM_SELECT_CAP0 );
            mcpwm_capture_enable( MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_NEG_EDGE, 0 );
        }
    }
    MCPWM0.int_clr.val = status; // handled
}

rmt_item32_t ultrasonic_trig_signal;

int measure_distance()
{
    mcpwm_capture_enable( MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0 );
    MCPWM0.int_ena.val = CAP0_INT_EN;
    CaptureVal capture;
    mcpwm_isr_register( MCPWM_UNIT_0, capture_handler, &capture, 0 /* ESP_INTR_FLAG_IRAM */, nullptr );

	rmt_write_items( ULTRASONIC_TRIG_CHANNEL, &ultrasonic_trig_signal, 1, false /* do not wait */ )
	    || Die( "Ultrasonic: trigger failed" );

    while ( !capture.done ) {
        std::this_thread::yield();
    }
    return capture.end - capture.start;
}

void ultrasonic_init_trig() {
    ultrasonic_trig_signal.duration0 = 1 + 20us / BASE_TICK;
    ultrasonic_trig_signal.level0 = 1;
    ultrasonic_trig_signal.duration1 = 1;
    ultrasonic_trig_signal.level1 = 0;

    rmt_config_t trig_config;
    trig_config.rmt_mode = RMT_MODE_TX;
    trig_config.channel = ULTRASONIC_TRIG_CHANNEL;
    trig_config.gpio_num = ULTRASONIC_TRIG;
    trig_config.mem_block_num = 1;
    trig_config.clk_div = 1;
    trig_config.tx_config.loop_en = false;
    trig_config.tx_config.carrier_en = false; // no modulation
    trig_config.tx_config.idle_output_en = true; // when idle…
    trig_config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW; // send 0

    rmt_config( &trig_config ) || Die( "Ultrasonic: trigger RMT failed to config" );
    rmt_driver_install( trig_config.channel, 0, 0 )
        || Die( "Ultrasonic: trigger RMT failed in driver install" );
}

extern "C" void app_main()
{
    mcpwm_gpio_init( MOTOR_MCPWM, MCPWM0A, MOTOR_PINS[0][0] );
    mcpwm_gpio_init( MOTOR_MCPWM, MCPWM0B, MOTOR_PINS[1][0] );
    OutPin< MOTOR_PINS[0][1] > motor0Base;
    OutPin< MOTOR_PINS[1][1] > motor1Base;
    mcpwm_gpio_init( MOTOR_MCPWM, MCPWM_CAP_0, ULTRASONIC_ECHO );
    gpio_pulldown_en( ULTRASONIC_ECHO );

    mcpwm_config_t pwm_config;
    pwm_config.frequency = MOTOR_MWM_FREQ.value;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    mcpwm_init( MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config );

    auto motorr_set = [&]( float speed ) {
        motor_set( motor0Base, MOTOR_MCPWM, MCPWM_TIMER_0, MCPWM_OPR_A, speed );
    };
    auto motorl_set = [&]( float speed ) {
        motor_set( motor1Base, MOTOR_MCPWM, MCPWM_TIMER_0, MCPWM_OPR_B, speed );
    };

    ultrasonic_init_trig();

    std::cout << "distance = " << measure_distance() << std::endl << std::flush;
}
