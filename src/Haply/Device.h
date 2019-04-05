/**
 **********************************************************************************************************************
 * @file       Device.h
 * @author     Colin Gallacher, Steven Ding, Christian Frisson
 * @version    V0.1.0
 * @date       01-March-2017
 * @brief      Device class definition
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */

#ifndef __HaplyDevice__
#define __HaplyDevice__

#include "Haply/Types.h"
#include "Haply/DeviceType.h"
#include "Haply/Actuator.h"
#include "Haply/Sensor.h"
#include "Haply/Board.h"
#include "Haply/Mechanisms.h"

namespace Haply
{

class Device
{

public:
    DeviceType       device_type;
public:
    byte 			       deviceID;

public:
    std::vector<Actuator>    	 motors;
public:
    std::vector<Sensor>		     encoders;

public:
    Board* 	      	 deviceLink;
public:
    Mechanisms* 		   mechanisms;

public:
    std::vector<byte> 			     actuator_positions;
public:
    std::vector<byte>			     encoder_positions;
private:
    byte 			       communicationType;
private:
    std::vector<float>   		   params;

    /**
    * Constructs a Device of the specified <code>DeviceType</code>, with the defined <code>deviceID</code>,
    * connected on the specified <code>Board</code>
    *
    * @param    device_type the degrees of freedom type of device to be implemented
    * @param    deviceID ID
    * @param    deviceLink: serial link used by device
    */
public:
    Device(DeviceType device_type, byte deviceID, Board* deviceLink);

    /**
    * Automatic setup of actuators and encoders based on setup parameters
    */
private:
    void device_component_auto_setup();

    /**
    * Set the indicated actuator to use the specified motor port
    *
    * @param    actuator index of actuator that needs parameter to be set or updated
    * @param    port specified motor port to be used (motor ports 1-4 on the Haply board)
    */
public:
    void set_actuator_parameters(int actuator, int port);

    /**
    * Set the indicated sensor (encoder) to use the initial offset, resolution, on the specified port
    *
    * @param    sensor index of sensor encoder that needs parameters to be set or updated
    * @param    offset initial offset in degrees that the encoder sensor should be initialized at
    * @param    resolution step resolution of the encoder sensor
    * @param    port specific motor port the encoder sensor is connect at (usually same as actuator)
    */
public:
    void set_encoder_parameters(int sensor, float offset, float resolution, int port);

    /**
    * Replaces the current Mechanisms that is being used with the specified Mechanisms
    *
    * @param    mechanisms new Mechanisms to replace the initialized or old Mechanisms currently in use
    */
public:
    void set_new_mechanism(Mechanisms* mechanisms);

    /**
    * Sets or updates device function parameters and loads frequency and amplitude vaues into params[]
    * (note* Hapkit specific function)
    *
    * @param    function device function to be carried out
    * @param    frequency frequency variable to be updated
    * @param    amplitude amplitude variable to be updated
    */
public:
    void set_parameters(byte function, float frequency, float amplitude);

    /**
    * Gathers all encoder sensor setup inforamation of all encoder sensors that are initialized and
    * sequentialy formats the data based on specified sensor index positions to send over serial port
    * interface for hardware device initialization
    */
public:
    void device_set_parameters();

    /**
    * hardware setup verification function (currently not used)
    */
public:
    void device_set_verification();

    /**
    * Requests encoder angle data from the hardware based on the initialized setup. function also
    * sends a torque output command of zero torque for each actuator in use
    */
public:
    void device_read_request();

    /**
    * Transmits specific torques that has been calculated and stored for each actuator over the serial
    * port interface
    */
public:
    void device_write_torques();

    /**
    * Transmits the contents of the params[] array over the serial port interface
    */
public:
    void send_data();

    /**
    * Receives angle position inforamation from the serial port interface and updates each indexed encoder sensor
    * to their respective received angle
    */
public:
    void device_read_angles();

    /**
    * Receives data from the serial port interface and updates parameters in mechanisms
    */
public:
    void receive_data();

    /**
    * assigns actuator positions based on actuator port
    */
private:
    void actuator_assignment(int actuator, Actuator m);

    /**
    * assigns encoder positions based on encoder port
    */
private:
    void encoder_assignment(int encoder, Sensor m);

    /**
    * Reads and update angles information from device hardware to encoders
    *
    * @returns    angles information received from device hardware
    */
public:
    std::vector<float> get_device_angles();

    /**
    * Reads and update angles information from device hardware to encoders and performs physics calculations
    *
    * @returns    end-effector coordinate position
    */
public:
    std::vector<float> get_device_position();

    /**
    * Performs physics calculations based on the given angle values
    *
    * @param      angles angles to be used for physics position calculation
    * @returns    end-effector coordinate position
    */
public:
    std::vector<float> get_device_position(std::vector<float> angles);

    /**
    * Calculates the needed output torques based on forces input and updates each initialized actuator
    * respectively
    *
    * @param     forces forces that need to be generated
    */
public:
    void set_device_torques(std::vector<float> forces);

    /**
     * Reads and update torques information from device hardware to encoders
     *
     * @param      angles angles to be used for physics position calculation
     * @returns    end-effector coordinate position
     */
public:
    std::vector<float> get_device_torques();
};

}

#endif // __HaplyDevice__
