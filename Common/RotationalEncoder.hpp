#pragma once

#include <cmath>
#include <cstdint>

namespace sjsu
{
    class RotationalEncoder
    {        
        public:

            virtual Returns<void> Initialize() = 0;
            virtual Returns<units::degrees> GetRotation() = 0;

            virtual Returns<void> Enable() = 0;
            virtual void SetAngleToZero() = 0;

    }
} 