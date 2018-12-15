#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/mcpwm.h>
#include <soc/soc.h>
#include <soc/mcpwm_struct.h>
#include <soc/mcpwm_reg.h>
#include <driver/spi_master.h>
#include "sdkconfig.h"
#include <chrono>
#include <thread>
#include "motor.hpp"
#include "ultrasonic.hpp"
#include <atomic>
#include <experimental/iterator>
#include <numeric>

static_assert( ATOMIC_INT_LOCK_FREE );

constexpr FreqHz BASE_CLOCK = FreqHz( APB_CLK_FREQ ); // 80 MHz
constexpr static std::chrono::nanoseconds BASE_TICK { 1000 * 1000 * 1000 / BASE_CLOCK.value };

constexpr gpio_num_t MOTOR_PINS[2][2] = { { 23_pin, 22_pin }, { 19_pin, 21_pin } };
constexpr auto MOTOR_MCPWM = MCPWM_UNIT_0;
constexpr FreqHz MOTOR_MWM_FREQ = 10_Hz;

constexpr gpio_num_t ULTRASONIC_TRIG = 18_pin;
constexpr static auto ULTRASONIC_TRIG_CHANNEL = RMT_CHANNEL_0;
constexpr static int ULTRASONIC_TRIG_DIV = 128;
constexpr static auto ULTRASONIC_TRIG_TICK = BASE_TICK * ULTRASONIC_TRIG_DIV;

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

struct UltrasonicDist {
    std::atomic< uint32_t > start{ 0 };
    std::atomic< uint32_t > distance{ 0 };
    std::atomic< uint32_t > cnt{ 0 };
};

rmt_item32_t ultrasonic_trig_signal[2];

void ultrasonic_trig( bool initial ) {
	rmt_write_items( ULTRASONIC_TRIG_CHANNEL, &ultrasonic_trig_signal[ initial ], 2 - initial,
                     false /* do not wait */ );
}

void capture_handler( void *arg ) {
    auto &dist = *reinterpret_cast< UltrasonicDist * >( arg );
    auto status = MCPWM0.int_st.val;

    if ( status & CAP0_INT_EN ) {
        auto edge = mcpwm_capture_signal_get_edge( MCPWM_UNIT_0, MCPWM_SELECT_CAP0 );
        if ( edge == 1 ) { // up edge
            dist.start = mcpwm_capture_signal_get_value( MCPWM_UNIT_0, MCPWM_SELECT_CAP0 );
            mcpwm_capture_enable( MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_NEG_EDGE, 0 );
        }
        else { // down edge
            auto end = mcpwm_capture_signal_get_value( MCPWM_UNIT_0, MCPWM_SELECT_CAP0 );
            dist.distance = std::chrono::duration_cast< std::chrono::milliseconds >(
                              (( end - dist.start ) * BASE_TICK * 340) / 2 ).count();
            dist.cnt++;
            mcpwm_capture_enable( MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0 );
            ultrasonic_trig( false );
        }
    }
    MCPWM0.int_clr.val = status; // handled
}

void ultrasonic_measure_continuosly( UltrasonicDist &dist ) {
    MCPWM0.int_ena.val = CAP0_INT_EN;
    mcpwm_isr_register( MCPWM_UNIT_0, capture_handler, &dist, 0 /* ESP_INTR_FLAG_IRAM */, nullptr );
    mcpwm_capture_enable( MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0 );
    ultrasonic_trig( true );
}

void ultrasonic_init_trig() {
    // do not measure faster than every 100ms
    ultrasonic_trig_signal[0].duration0 = 50ms / ULTRASONIC_TRIG_TICK;
    ultrasonic_trig_signal[0].level0 = 0;
    ultrasonic_trig_signal[0].duration1 = 50ms / ULTRASONIC_TRIG_TICK;
    ultrasonic_trig_signal[0].level1 = 0;
    ultrasonic_trig_signal[1].duration0 = 1 + 20us / ULTRASONIC_TRIG_TICK;
    ultrasonic_trig_signal[1].level0 = 1;
    ultrasonic_trig_signal[1].duration1 = 1;
    ultrasonic_trig_signal[1].level1 = 0;

    rmt_config_t trig_config;
    trig_config.rmt_mode = RMT_MODE_TX;
    trig_config.channel = ULTRASONIC_TRIG_CHANNEL;
    trig_config.gpio_num = ULTRASONIC_TRIG;
    trig_config.mem_block_num = 1;
    trig_config.clk_div = ULTRASONIC_TRIG_DIV;
    trig_config.tx_config.loop_en = false;
    trig_config.tx_config.carrier_en = false; // no modulation
    trig_config.tx_config.idle_output_en = true; // when idle…
    trig_config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW; // send 0

    rmt_config( &trig_config ) || Die( "Ultrasonic: trigger RMT failed to config" );
    rmt_driver_install( trig_config.channel, 0, 0 )
        || Die( "Ultrasonic: trigger RMT failed in driver install" );
}

constexpr gpio_num_t LINE_CS = 14_pin;
constexpr gpio_num_t LINE_MOSI = 27_pin;
constexpr gpio_num_t LINE_MISO = 26_pin;
constexpr gpio_num_t LINE_SCK = 25_pin;

constexpr gpio_num_t LED_FEEDER_SCK = 0_pin;
constexpr gpio_num_t LED_FEEDER_MOSI = 4_pin;
constexpr gpio_num_t LED_FEEDER_SET = 16_pin;

spi_device_handle_t init_line() {
    spi_bus_config_t spi_bus = {
        .mosi_io_num = LINE_MOSI,
        .miso_io_num = LINE_MISO,
        .sclk_io_num = LINE_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
        .flags = 0
    };
    spi_bus_initialize( HSPI_HOST, &spi_bus, 0 ) || Die( "LINE: failed to initialize SPI bus" );

    spi_device_handle_t dev;
    spi_device_interface_config_t dev_cfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 1_MHz .value,
//        .input_delay_ns = 0,
        .spics_io_num = LINE_CS,
        .flags = 0,
        .queue_size = 2,
        .pre_cb = 0,
        .post_cb = 0
    };
    spi_bus_add_device( HSPI_HOST, &dev_cfg, &dev ) || Die( "LINE: failed to initialize SPI device" );
    return dev;
}

struct LineData
{
    bool operator[]( int ix ) const {
        return raw & (1 << ix);
    }

    struct const_iterator
    {
        const_iterator( const LineData *self = nullptr, int ix = 8 ) : _self( self ), _ix( ix ) { }

        bool operator*() const { return (*_self)[ _ix ]; }
        const_iterator &operator++() { ++_ix; return *this; }

        bool operator==( const_iterator &o ) const { return _ix == o._ix; }
        bool operator!=( const_iterator &o ) const { return _ix != o._ix; }

        const LineData *_self;
        int _ix;
    };
    using iterator = const_iterator;

    const_iterator begin() const { return const_iterator( this, 0 ); }
    const_iterator end() const { return const_iterator( this, 8 ); }

    int diretion() const {
        return std::inner_product( begin(), end(), WEIGHTS.begin(), 0 );
    }

    static constexpr std::array< int, 8 > WEIGHTS = { -16, -14, -12, 4,   4, 12, 14, 16 };
    static constexpr int MAX = WEIGHTS[7];
    uint8_t raw{};
};

LineData read_line( spi_device_handle_t dev )
{
    LineData values;
    for ( int i = 0; i < 8; ++i ) {
        spi_transaction_t trans{};
        trans.tx_data[ 0 ] = 0b00000001;
        trans.tx_data[ 1 ] = 0b10000000 | i << 4;
        trans.length = 24;
        trans.rxlength = 24;
        trans.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;

        spi_device_transmit( dev, &trans );

        values.raw |= (((trans.rx_data[1] & 0b11) << 8 | trans.rx_data[2]) > 512) << i;
    }
    return values;
}

spi_device_handle_t init_led_feeder() {
    spi_bus_config_t spi_bus = {
        .mosi_io_num = LED_FEEDER_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = LED_FEEDER_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
        .flags = 0
    };
    spi_bus_initialize( VSPI_HOST, &spi_bus, 0 ) || Die( "LED FEEDER: failed to initialize SPI bus" );

    spi_device_handle_t dev;
    spi_device_interface_config_t dev_cfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 1_MHz .value,
        .spics_io_num = -1,
        .flags = 0,
        .queue_size = 2,
        .pre_cb = 0,
        .post_cb = 0
    };
    spi_bus_add_device( VSPI_HOST, &dev_cfg, &dev ) || Die( "LED_FEEDER: failed to initialize SPI device" );
    return dev;
}

void set_leds( spi_device_handle_t dev, LineData data ) {
    spi_transaction_t trans{};
    trans.length = 8;
    trans.rxlength = 0;
    trans.tx_data[ 0 ] = data.raw;
    trans.flags = SPI_TRANS_USE_TXDATA;
    spi_device_transmit( dev, &trans );

    static OutPin< LED_FEEDER_SET > set;
    set.set( true );
    set.set( false );
}

struct LineSamples
{
    int proportional() const { return samples[ idx ].second.diretion(); }

    int derivative() const {
        int prev = samples[ (idx - 1) % samples.size() ].second.diretion();
        return (proportional() - prev) / 2;
    }

    int integrative() const {
        int cnt = 0;
        return std::accumulate( samples.begin(), samples.end(), 0,
                      [&]( int accum, auto data ) {
                          if ( data.first ) {
                              ++cnt;
                              return accum + data.second.diretion();
                          }
                          return accum;
                      } )
                  / cnt;
    }

    LineData any() const {
        LineData out;
        for ( auto x : samples )
            out.raw |= x.second.raw;
        return out;
    }

    std::pair< float, float > speeds() const {
        constexpr float CORRECTION_MIN_SPEED = 20;
        if ( samples[ idx ].second.raw == 0 )
            return { 0, 0 };

        constexpr int COEFF[3] = { 8, 2, 2 };
        int sum = (COEFF[0] * proportional() + COEFF[1] * derivative() + COEFF[2] * integrative())
                  / (abs( COEFF[0] ) + abs( COEFF[1] ) + abs( COEFF[2] ) );
        if ( sum == 0 )
            return { 100, 100 };

        // > 0 -> too much right -> correct to the left -> slow right
        if ( sum > 0 )
            sum = std::min( sum, LineData::MAX );
        else
            sum = std::max( sum, -LineData::MAX );
        auto corr = (sum * ((100.0 - CORRECTION_MIN_SPEED) / LineData::MAX));
        return { 100 + corr, 100 - corr };
    }

    void add( LineData data ) {
        sample.raw |= data.raw;
        ++sampled;
        if ( sampled >= ADD_SAMPLES ) {
            sample.raw = sampled = 0;
            ++idx;
            idx %= samples.size();
            samples[ idx ].first = true;
            samples[ idx ].second = data;
        }
    };

    static constexpr int ADD_SAMPLES = 5;
    LineData sample{};
    int8_t sampled = 0;
    std::array< std::pair< bool, LineData >, 16 > samples{};
    unsigned idx = 0;
};

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
    UltrasonicDist dist;
    ultrasonic_measure_continuosly( dist );

    auto line_sensor = init_line();
    auto led_feeder = init_led_feeder();

    using time = std::chrono::steady_clock;

    LineSamples line_samples;

  wait:
    std::cout << "waiting\n" << std::flush;
    bool near = false;
    while ( true ) {
        auto next = time::now() + 50ms;

        if ( !near && dist.distance < 200 && dist.distance > 0 )
            near = true;
        if ( near && dist.distance > 200 )
            break;
        auto vals = read_line( line_sensor );
        set_leds( led_feeder, vals );
        std::cout << " dist = " << dist.distance
                  << std::endl;

        std::this_thread::sleep_until( next );
    }
    std::cout << "start\n" << std::flush;

    auto next = time::now() + 10ms;
    while ( true ) {
        for ( int i = 0; i < LineSamples::ADD_SAMPLES; ++i ) {
            std::this_thread::sleep_until( next );
            next += 20ms;

            auto vals = read_line( line_sensor );
            line_samples.add( vals );
            set_leds( led_feeder, vals );
        }

        auto speeds = line_samples.speeds();
        std::cout << "p = " << line_samples.proportional()
                  << " i = " << line_samples.integrative()
                  << " d = " << line_samples.derivative()
                  << " L = " << speeds.first
                  << " R = " << speeds.second
                  << " dist = " << dist.distance
                  << std::endl;
        motorl_set( speeds.first * 0.25 );
        motorr_set( speeds.second * 0.25 );
        if ( line_samples.any().raw == 0 )
            goto wait;

//        std::cout << std::chrono::duration_cast< std::chrono::milliseconds >( next - time::now() ).count() << "ms\n" << std::flush;
    }
}
