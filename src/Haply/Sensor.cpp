/**
 **********************************************************************************************************************
 * @file       Sensor.cpp
 * @author     Colin Gallacher, Steven Ding, Christian Frisson
 * @version    V0.1.0
 * @date       01-March-2017
 * @brief      Sensor class definition
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */

#include "Haply/Sensor.h"

using namespace Haply;

/**
 * Constructs a Sensor with the given motor port position, to be initialized with the given angular offset,
 * at the specified step resoluiton
 *
 * @param    offset initial offset in degrees that the encoder sensor should be initialized at
 * @param    resolution step resolution of the encoder sensor
 * @param    port specific motor port the encoder sensor is connect at (usually same as actuator)
 */

Sensor::Sensor(float offset, float resolution, int port)
{
    this->encoder_offset = offset;
    this->encoder_resolution = resolution;
    this->encoder_port = port;
    float   encoder_offset	      = 0;
    float   encoder_resolution    = 0;
    float   angle 			          = 0;
    int     encoder_port	 	      = 0;
}


/**
 * Set offset parameter of sensor
 *
 * @param    offset initial angular offset in degrees
 */

void Sensor::set_offset(float offset)
{
    encoder_offset = offset;
}


/**
 * Set resolution parameter of sensor
 *
 * @param    resolution step resolution of encoder sensor
 */

void Sensor::set_resolution(float resolution)
{
    encoder_resolution = resolution;
}


/**
 * Set motor port position to be used by sensor
 *
 * @param    port motor port position (motor port connection on Haply board)
 */

void Sensor::set_port(int port)
{
    encoder_port = port;
}


/**
 * Set angle variable to the specified angle
 *
 * @param    angle angle value
 */

void Sensor::set_angle(float angle)
{
    this->angle = angle;
}


/**
 * @return    current offset parameter
 */

float Sensor::get_offset()
{
    return encoder_offset;
}


/**
 * @return    encoder resolution of encoder sensor being used
 */

float Sensor::get_resolution()
{
    return encoder_resolution;
}


/**
 * @return    current motor port position
 */

int Sensor::get_port()
{
    return encoder_port;
}


/**
 * @return    current angle information
 */

float Sensor::get_angle()
{
    return angle;
}


