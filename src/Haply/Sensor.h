/**
 **********************************************************************************************************************
 * @file       Sensor.h
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

#ifndef __HaplySensor__
#define __HaplySensor__

namespace Haply
{

class Sensor
{

private:

    int     encoder;
    int     direction;
    float   encoder_offset;
    float   encoder_resolution;
    float   value;
    int     port;

    /**
     * Constructs a Sensor set using motor port position one
     */
public:
    Sensor();

    /**
    * Constructs a Sensor with the given motor port position, to be initialized with the given angular offset,
    * at the specified step resolution (used for construction of encoder sensor)
    *
    * @param    encoder encoder index
    * @param    offset initial offset in degrees that the encoder sensor should be initialized at
    * @param    resolution step resolution of the encoder sensor
    * @param    port specific motor port the encoder sensor is connect at (usually same as actuator)
    */
public:
    Sensor(int encoder, int direction, float offset, float resolution, int port);

    /**
     * Set encoder index parameter of sensor
     *
     * @param    encoder index
     */
public:
    void set_encoder(int encoder);

    /**
     * Set encoder direction of detection
     *
     * @param    encoder index
     */
public:
    void set_direction(int direction);

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
     * Set sensor value variable to the specified input
     *
     * @param    value sensor value
     */
public:
    void set_value(float value);


    /**
     * @return    encoder index
     */
public:
    int get_encoder();


    /**
     * @return    encoder direction
     */
public:
    int get_direction();


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
     * @return    current sensor value information
     */
public:
    float get_value();

}; // class Sensor

} // namespace Haply

#endif // __HaplySensor__
