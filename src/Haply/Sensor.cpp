/**
 **********************************************************************************************************************
 * @file       Sensor.cpp
 * @author     Colin Gallacher, Steven Ding, Christian Frisson
 * @version    V2.1.0
 * @date       23-April-2019
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
 * Constructs a Sensor set using motor port position one
 */

Sensor::Sensor()
    : Sensor(0, 0, 0, 0, 0)
{

}

/**
 * Constructs a Sensor with the given motor port position, to be initialized with the given angular offset,
 * at the specified step resolution (used for construction of encoder sensor)
 *
 * @param    encoder encoder index
 * @param    offset initial offset in degrees that the encoder sensor should be initialized at
 * @param    resolution step resolution of the encoder sensor
 * @param    port specific motor port the encoder sensor is connect at (usually same as actuator)
 */

Sensor::Sensor(int encoder, int direction, float offset, float resolution, int port)
{
    this->encoder = encoder;
    this->direction = direction;
    this->encoder_offset = offset;
    this->encoder_resolution = resolution;
    this->port = port;
}

/**
 * Set encoder index parameter of sensor
 *
 * @param    encoder index
 */

void Sensor::set_encoder(int encoder)
{
    this->encoder = encoder;
}

/**
 * Set encoder direction of detection
 *
 * @param    encoder index
 */

void Sensor::set_direction(int direction)
{
    this->direction = direction;
}

/**
 * Set offset parameter of sensor
 *
 * @param    offset initial angular offset in degrees
 */

void Sensor::set_offset(float offset)
{
    this->encoder_offset = offset;
}


/**
 * Set resolution parameter of sensor
 *
 * @param    resolution step resolution of encoder sensor
 */

void Sensor::set_resolution(float resolution)
{
    this->encoder_resolution = resolution;
}


/**
 * Set motor port position to be used by sensor
 *
 * @param    port motor port position (motor port connection on Haply board)
 */

void Sensor::set_port(int port)
{
    this->port = port;
}


/**
 * Set sensor value variable to the specified input
 *
 * @param    value sensor value
 */

void Sensor::set_value(float value)
{
    this->value = value;
}


/**
 * @return    encoder index
 */

int Sensor::get_encoder()
{
    return encoder;
}


/**
 * @return    encoder direction
 */

int Sensor::get_direction()
{
    return direction;
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
    return port;
}


/**
 * @return    current sensor value information
 */

float Sensor::get_value()
{
    return value;
}
