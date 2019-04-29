/**
 **********************************************************************************************************************
 * @file       Pwm.h
 * @author     Steve Ding, Colin Gallacher, Christian Frisson
 * @version    V1.1.0
 * @date       23-April-2019
 * @brief      Pwm class definition
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */

#ifndef __HaplyPwm__
#define __HaplyPwm__

namespace Haply
{

class Pwm
{

private:
    int       pin;
    int       value;

    /**
     * Constructs an empty PWM output for use
     */
public:
    Pwm();

    /**
     * Constructs a PWM output at the specified pin and at the desired percentage
     *
     *	@param	pin pin to output pwm signal
     * @param 	pulseWidth percent of pwm output, value between 0 to 100
     */
public:
    Pwm(int pin, float pulseWidth);

    /**
     * Set pin parameter of pwm
     *
     * @param	pin pin to output pwm signal
     */
public:
    void set_pin(int pin);

    /**
     * Set value variable of pwm
     */
public:
    void set_pulse(float percent);

    /**
     * @return	pin pwm signal is outputting
     */
public:
    int get_pin();

    /**
     * @return raw value of pwm signal between 0 to 255
     */
public:
    int get_value();

    /**
     * @return percent value of pwm signal
     */
public:
    float get_pulse();

}; // class Pwm

} // namespace Haply

#endif // __HaplyPwm__
