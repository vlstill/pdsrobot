#pragma once

namespace axx {

// http://www.mstarlabs.com/apeng/techniques/pidsoftw.html
// modified to work with integers instead of floating point numbers
template< long Gain, long IntTime, long DiffTime, long DeltaT,
          long Setpoint = 0, // the value to maintain
          long PrecBin = 8
        >
struct PID
{

    PID() : integral( 0 ), derivative( Setpoint * DIVISOR )
    { }

    long operator()( long in ) { return update( in ); }

    long update( long in )
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
    static constexpr long DIVISOR = 1 << PrecBin;

    long integral;
    long derivative;
};

}
