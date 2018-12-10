#pragma once

namespace axx {

// http://www.mstarlabs.com/apeng/techniques/pidsoftw.html
// modified to work with integers instead of floating point numbers
template< typename T,
          T Gain, T IntTime, T DiffTime, T DeltaT,
          T Setpoint = 0, // the value to maintain
          T PrecBin = 8
        >
struct PID
{

    PID() : integral( 0 ), derivative( Setpoint * DIVISOR )
    { }

    T operator()( T in ) { return update( in ); }

    T update( T in )
    {
        auto err = in - Setpoint;
        auto out = err * DIVISOR;

        out += integral * IntTime / DeltaT;
        integral += err;

        out += (err - derivative) * DiffTime / DeltaT;
        derivative = err;

        return (- Gain * out) / DIVISOR;
    }

  private:
    static constexpr T DIVISOR = 1 << PrecBin;

    T integral;
    T derivative;
};

}
