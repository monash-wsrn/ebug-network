#ifndef EBUG_POLOLU_WHEEL_H
#define EBUG_POLOLU_WHEEL_H

#include <string>
#include <cmath>

namespace ebug
{
    class PololuWheel
    {
    public:
        std::string name = "";

        double pos = 0;     // State interface (position)
        double vel = 0;     // State interface (velocity)

        double cmd = 0;     // Command interface (target velocity)
        
        
        int enc_prev = 0;   // 
        int enc_curr = 0;   // Encoder ticks

        double rads_per_count = 0;

    public:
        void configure(const std::string &name, int cpr)
        {
            this->name = name;
            this->rads_per_count = (2.0 * M_PI) / (double) cpr;
        }

        void recalculate(int16_t v, const double dt)
        {   
            enc_prev = enc;
            enc_curr = (int) v;

            double prev = pos;
            pos = enc * rads_per_count;
            vel = (pos - prev) / dt;
        }

        int16_t target(const float loop_rate)
        {
            return (int16_t) ((float) cmd / (float) rads_per_count / loop_rate) 
        }

        PololuWheel() = default;
        PololuWheel(const std::string &name, int cpr)
        {
            configure(name, cpr);
        }
    };
}

#endif // EBUG_POLOLU_WHEEL_H