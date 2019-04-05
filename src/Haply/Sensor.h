/**
 **********************************************************************************************************************
 * @file       Sensor.h
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

#ifndef __HaplySensor__
#define __HaplySensor__

namespace Haply
{

class Sensor
{

private:
    float encoder_offset;
    float encoder_resolution;
    float angle;
    int encoder_port;

    /**
       * Constructs a Sensor with the given motor port position, to be initialized with the given angular offset,
       * at the specified step resoluiton
       *
       * @param    offset initial offset in degrees that the encoder sensor should be initialized at
       * @param    resolution step resolution of the encoder sensor
       * @param    port specific motor port the encoder sensor is connect at (usually same as actuator)
       */
public:
    Sensor(float offset = 0, float resolution = 0, int port = 1);

    /**
       * Set offset parameter of sensor
       *
       * @param    offset initial angular offset in degrees
       */
public:
    void set_offset(float offset);

    /**
       * Set resolution parameter of sensor
       *
       * @param    resolution step resolution of encoder sensor
       */
public:
    void set_resolution(float resolution);

    /**
       * Set motor port position to be used by sensor
       *
       * @param    port motor port position (motor port connection on Haply board)
       */
public:
    void set_port(int port);

    /**
       * Set angle variable to the specified angle
       *
       * @param    angle angle value
       */
public:
    void set_angle(float angle);

    /**
       * @return    current offset parameter
       */
public:
    float get_offset();

    /**
       * @return    encoder resolution of encoder sensor being used
       */
public:
    float get_resolution();

    /**
       * @return    current motor port position
       */
public:
    int get_port();

    /**
       * @return    current angle information
       */
public:
    float get_angle();
};

} // namespace Haply

#endif // __HaplySensor__
