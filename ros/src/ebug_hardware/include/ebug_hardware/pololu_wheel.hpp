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
        int enc = 0;
        double cmd = 0;
        double pos = 0;
        double vel = 0;
        double eff = 0;
        double velSetPt = 0;
        double rads_per_count = 0;

    public:
        void configure(const std::string &name, int cpr)
        {
            this->name = name;
            this->rads_per_count = (2.0 * M_PI) / (double) cpr;
        }

        void recalculate(int16_t v, const double dt)
        {   
            enc = (int) v;

            double prev = pos;
            pos = enc * rads_per_count;
            vel = (pos - prev) / dt;
        }

        PololuWheel() = default;
        PololuWheel(const std::string &name, int cpr)
        {
            configure(name, cpr);
        }
    };
}

#endif // EBUG_POLOLU_WHEEL_H