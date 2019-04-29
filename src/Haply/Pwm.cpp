/**
 **********************************************************************************************************************
 * @file       Pwm.cpp
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

#include "Pwm.h"

using namespace Haply;

/**
 * Constructs an empty PWM output for use
 */
Pwm::Pwm()
    : Pwm(0, 0)
{

}

/**
 * Constructs a PWM output at the specified pin and at the desired percentage
 *
 *	@param	pin pin to output pwm signal
 * @param 	pulseWidth percent of pwm output, value between 0 to 100
 */
Pwm::Pwm(int pin, float pulseWidth)
    :pin(0),value(0)
{
    this->pin = pin;

    if(pulseWidth > 100.0) {
        this->value = 255;
    } else {
        this->value = (int)(pulseWidth * 255 / 100);
    }
}


/**
 * Set pin parameter of pwm
 *
 * @param	pin pin to output pwm signal
 */
void Pwm::set_pin(int pin)
{
    this->pin = pin;
}


/**
 * Set value variable of pwm
 */
void Pwm::set_pulse(float percent)
{

    if(percent > 100.0) {
        this->value = 255;
    }	else if(percent < 0) {
        this->value = 0;
    } else {
        this->value = (int)(percent * 255 / 100);
    }
}


/**
 * @return	pin pwm signal is outputting
 */
int Pwm::get_pin()
{
    return pin;
}


/**
 * @return raw value of pwm signal between 0 to 255
 */
int Pwm::get_value()
{
    return value;
}


/**
 * @return percent value of pwm signal
 */
float Pwm::get_pulse()
{

    float percent = value * 100 / 255;

    return percent;
}
