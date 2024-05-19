#ifndef EBUG_POLOLU_LIGHTS_H
#define EBUG_POLOLU_LIGHTS_H

#include <string>

namespace ebug
{
    class PololuLights
    {
    public:
        std::string name = "";
        uint32_t RGBA;

    public:
        void configure(const std::string &name, uint32_t RGBA)
        {
            this->name = name;
            this->RGBA = RGBA;
        }
        
        PololuLights() = default;
        PololuLights(const std::string &name, uint32_t RGBA)
        {
            configure(name, RGBA);
        }

    };
}

#endif // EBUG_POLOLU_LIGHTS_H