/**
 **********************************************************************************************************************
 * @file       Actuator.h
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

#ifndef __HaplyActuator__
#define __HaplyActuator__

namespace Haply
{

class Actuator
{

private:
    float   torque;
    int     actuator_port;

    /**
     * Creates an Actuator using motor port position 1
     */
public:
    Actuator();

    /**
     * Creates an Actuator using the given motor port position
     *
     * @param    port motor port position for actuator
     */
public:
    Actuator(int port);

    /**
     * Sets motor port position to be used by Actuator
     *
     * @param   port motor port position
     */
public:
    void set_port(int port);

    /**
     * Sets torque variable to the given torque value
     *
     * @param   torque new torque value for update
     */
public:
    void set_torque(float torque);

    /**
     * @return   current motor port position in use
     */
public:
    int get_port();

    /**
     * @return   current torque information
     */
public:
    float get_torque();

};

}

#endif // __HaplyActuator__
