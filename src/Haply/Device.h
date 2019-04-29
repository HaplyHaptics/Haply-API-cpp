/**
 **********************************************************************************************************************
 * @file  Device.h
 * @author     Colin Gallacher, Steven Ding, Christian Frisson
 * @version    V2.1.0
 * @date  17-April-2019
 * @brief Device class definition
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */

#ifndef __HaplyDevice__
#define __HaplyDevice__

#include "Haply/Actuator.h"
#include "Haply/Board.h"
#include "Haply/Mechanisms.h"
#include "Haply/Pwm.h"
#include "Haply/Sensor.h"
#include "Haply/Types.h"

namespace Haply
{

class Device
{

private:
    Board* device_link;

    byte device_id;
    Mechanisms* mechanism;

    byte communication_type;

    int  actuators_active;
    std::vector<Actuator> motors;

    int  encoders_active;
    std::vector<Sensor> encoders;

    int  sensors_active;
    std::vector<Sensor> sensors;

    int  pwms_active;
    std::vector<Pwm> pwms;

    std::vector<byte> actuator_positions;
    std::vector<byte> encoder_positions;

    /**
    * Constructs a Device of the specified <code>device_id</code>, connected on the specified <code>Board</code>
    *
    * @param    device_id ID
    * @param    device_link: serial link used by device
    */
public:
    Device(byte device_id, Board* device_link);

    // device setup functions
    /**
     * add new actuator to platform
     *
     * @param    actuator index of actuator (and index of 1-4)
     * @param    roatation positive direction of actuator rotation
     * @param    port specified motor port to be used (motor ports 1-4 on the Haply board)
     */
public:
    void add_actuator(int actuator, int rotation, int port);

    /**
     * Add a new encoder to the platform
     *
     * @param    actuator index of actuator (an index of 1-4)
     * @param    positive direction of rotation detection
     * @param    offset encoder offset in degrees
     * @param    resolution encoder resolution
     * @param    port specified motor port to be used (motor ports 1-4 on the Haply board)
     */
public:
    void add_encoder(int encoder, int rotation, float offset, float resolution, int port);

    /**
     * Add an analog sensor to platform
     *
     * @param    pin the analog pin on haply board to be used for sensor input (Ex: A0)
     */
public:
    void add_analog_sensor(std::string pin);

    /**
     * Add a PWM output pin to the platform
     *
     * @param		pin the pin on the haply board to use as the PWM output pin
     */
public:
    void add_pwm_pin(int pin);


    /**
     * Set the device mechanism that is to be used
     *
     * @param    mechanisms new Mechanisms for use
     */
public:
    void set_mechanism(Mechanisms* mechanisms);

    /**
     * Gathers all encoder, sensor, pwm setup information of all encoders, sensors, and pwm pins that are
     * initialized and sequentialy formats the data based on specified sensor index positions to send over
     * serial port interface for hardware device initialization
     */
public:
    void device_set_parameters();

    /**
     * assigns actuator positions based on actuator port
     */
private:
    void actuator_assignment(int actuator, int port);

    /**
     * assigns encoder positions based on actuator port
     */
private:
    void encoder_assignment(int encoder, int port);


    // device communication functions
    /**
     * Receives angle position and sensor inforamation from the serial port interface and updates each indexed encoder
     * sensor to their respective received angle and any analog sensor that may be setup
     */
public:
    void device_read_data();

    /**
     * Requests data from the hardware based on the initialized setup. function also sends a torque output
     * command of zero torque for each actuator in use
     */
public:
    void device_read_request();

    /**
     * Transmits specific torques that has been calculated and stored for each actuator over the serial
     * port interface, also transmits specified pwm outputs on pwm pins
     */
public:
    void device_write_torques();

    /**
     * Set pulse of specified PWM pin
     */
public:
    void set_pwm_pulse(int pin, float pulse);

    /**
     * Gets percent PWM pulse value of specified pin
     */
public:
    float get_pwm_pulse(int pin);

    /**
     * Gathers current state of angles information from encoder objects
     *
     * @returns    most recent angles information from encoder objects
     */
public:
    std::vector<float> get_device_angles();

    /**
     * Gathers current data from sensor objects
     *
     * @returns    most recent analog sensor information from sensor objects
     */
public:
    std::vector<float> get_sensor_data();

    /**
     * Performs physics calculations based on the given angle values
     *
     * @param angles angles to be used for physics position calculation
     * @returns    end-effector coordinate position
     */
public:
    std::vector<float> get_device_position(std::vector<float> angles);

    /**
     * Calculates the needed output torques based on forces input and updates each initialized
     * actuator respectively
     *
     * @param     forces forces that need to be generated
     * @returns   torques that need to be outputted to the physical device
     */
public:
    std::vector<float> set_device_torques(std::vector<float> forces);

}; // class Device

} // namespace Haply

#endif // __HaplyDevice__
