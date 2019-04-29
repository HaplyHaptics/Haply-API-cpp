/**
 **********************************************************************************************************************
 * @file       Actuator.cpp
 * @author     Colin Gallacher, Steven Ding, Christian Frisson
 * @version    V2.1.0
 * @date       15-April-2019
 * @brief      Actuator class definition
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */

#include "Haply/Actuator.h"

using namespace Haply;

/**
   * Creates an Actuator using motor port position 1
   */
Actuator::Actuator()
{
    Actuator(0,0,0);
}

/**
   * Creates an Actuator using the given motor port position
   *
   * @param    actuator actuator index
   * @param    direction for actuator
   * @param    port motor port position for actuator
   */
Actuator::Actuator(int actuator, int direction, int port)
    : actuator(0), direction(0), torque(0), actuator_port(0)
{
    this->actuator = actuator;
    this->direction = direction;
    this->actuator_port = port;
}


/**
 * Set actuator index parameter of sensor
 *
 * @param    actuator index
 */
void Actuator::set_actuator(int actuator)
{
    this->actuator = actuator;
}


/**
 * Set actuator rotation direction
 *
 * @param    direction of rotation
 */
void Actuator::set_direction(int direction)
{
    this->direction = direction;
}

/**
   * Sets motor port position to be used by Actuator
   *
   * @param   port motor port position
   */
void Actuator::set_port(int port)
{
    actuator_port = port;
}

/**
   * Sets torque variable to the given torque value
   *
   * @param   torque new torque value for update
   */
void Actuator::set_torque(float torque)
{
    this->torque = torque;
}

/**
 * @return    actuator index
 */
int Actuator::get_actuator()
{
    return actuator;
}

/**
 * @return    actuator direction
 */
int Actuator::get_direction()
{
    return direction;
}

/**
   * @return   current motor port position in use
   */
int Actuator::get_port()
{
    return actuator_port;
}

/**
   * @return   current torque information
   */
float Actuator::get_torque()
{
    return torque;
}