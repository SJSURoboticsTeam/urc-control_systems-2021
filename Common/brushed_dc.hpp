#pragma once

namespace sjsu
{
    class brushed_dc
    {
    public:
        /// Initialize peripherals required to communicate with device. Must be the
        /// first method called on this driver. After this is called, `Enable()` can
        /// be called.
        virtual bool Initilize() = 0;
        /// This method must be called before running `Read()`. This function will
        /// configure the device settings as defined through the constructor of this
        /// interface's implementation. Some implementations have more detail or
        /// settings than others.
        virtual bool Enable() = 0;
        virtual bool DriveForward() = 0;
        virtual bool DriveBackward() = 0;

    private:
        const sjsu::Pwm &pwm;
        const sjsu::Gpio &dir;
        const sjsu::Gpio &brake;

} // namespace sjsu