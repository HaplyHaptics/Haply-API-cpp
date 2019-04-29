/**
 **********************************************************************************************************************
 * @file       Actuator.h
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

#ifndef __HaplyActuator__
#define __HaplyActuator__

namespace Haply
{

class Actuator
{

private:
    int     actuator;
    int     direction;
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
     * @param	 actuator actuator index
     * @param    direction for actuator
     * @param    port motor port position for actuator
     */
public:
    Actuator(int actuator, int direction, int port);

    /**
     * Set actuator index parameter of sensor
     *
     * @param    actuator index
     */
public:
    void set_actuator(int actuator);

    /**
     * Set actuator rotation direction
     *
     * @param    direction of rotation
     */
public:
    void set_direction(int direction);

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
     * @return    actuator index
     */
public:
    int get_actuator();

    /**
     * @return    actuator direction
     */
public:
    int get_direction();

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

}; // class Actuator

} // namespace Haply

#endif // __HaplyActuator__
