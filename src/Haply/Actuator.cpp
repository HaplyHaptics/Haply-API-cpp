/**
 **********************************************************************************************************************
 * @file       Actuator.cpp
 * @author     Colin Gallacher, Steven Ding, Christian Frisson
 * @version    V0.1.0
 * @date       01-March-2017
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
    Actuator(1);
}

/**
   * Creates an Actuator using the given motor port position
   *
   * @param    port motor port position for actuator
   */
Actuator::Actuator(int port)
    : torque(0), actuator_port(0)
{
    this->actuator_port = port;
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