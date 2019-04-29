/**
 **********************************************************************************************************************
 * @file       Device.cpp
 * @author     Colin Gallacher, Steven Ding, Christian Frisson
 * @version    V2.1.0
 * @date       17-April-2019
 * @brief      Device class definition
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */

#include "Haply/Device.h"

#include <algorithm>

using namespace Haply;

/**
    * Constructs a Device of the specified <code>device_id</code>, connected on the specified <code>Board</code>
    *
    * @param    device_id ID
    * @param    device_link: serial link used by device
    */

Device::Device(byte device_id, Board *device_link)
    : device_link(0),
      device_id(0), mechanism(0),
      communication_type(0),
      actuators_active(0),
      encoders_active(0),
      sensors_active(0),
      pwms_active(0)
{
    this->actuator_positions = std::vector<byte>(4, 0);
    this->encoder_positions = std::vector<byte>(4, 0);
    this->device_id = device_id;
    this->device_link = device_link;
}



// device setup functions
/**
 * add new actuator to platform
 *
 * @param    actuator index of actuator (and index of 1-4)
 * @param    roatation positive direction of actuator rotation
 * @param    port specified motor port to be used (motor ports 1-4 on the Haply board)
 */
void Device::add_actuator(int actuator, int rotation, int port)
{
    bool error = false;

    if (port <= 0 || port > 4) {
        std::cerr << "error: actuator position index out of bounds!" << std::endl;
        error = true;
    }

    if(actuator < 1 || actuator > 4) {
        std::cerr << "error: actuator index out of bounds!" << std::endl;
        error = true;
    }

    // determine index for copying
    int j = 0;
    for(int i = 0; i < actuators_active; i++) {
        if(motors[i].get_actuator() < actuator) {
            j++;
        }

        if(motors[i].get_actuator() == actuator) {
            std::cerr << "error: actuator " << actuator << " has already been set" << std::endl;
            error = true;
        }
    }

    if(!error) {
        std::vector<Actuator> temp(actuators_active + 1);

        std::copy( motors.begin() + 0, motors.begin() + 0 + motors.size(), temp.begin());

        if(j < actuators_active) {
            std::copy( motors.begin() + j, motors.begin() + j+1 + motors.size() - j, temp.begin());
        }

        temp[j] = Actuator(actuator, rotation, port);
        actuator_assignment(actuator, port);

        this->motors = temp;
        actuators_active++;
    }
}

/**
 * Add a new encoder to the platform
 *
 * @param    actuator index of actuator (an index of 1-4)
 * @param    positive direction of rotation detection
 * @param    offset encoder offset in degrees
 * @param    resolution encoder resolution
 * @param    port specified motor port to be used (motor ports 1-4 on the Haply board)
 */
void Device::add_encoder(int encoder, int rotation, float offset, float resolution, int port)
{
    bool error = false;

    if (port <= 0 || port > 4) {
        std::cerr << "error: encoder position index out of bounds!" << std::endl;
    }

    if(encoder < 1 || encoder > 4) {
        std::cerr << "error: encoder index out of bounds!" << std::endl;
        error = true;
    }

    // determine index for copying
    int j = 0;
    for(int i = 0; i < encoders_active; i++) {
        if(encoders[i].get_encoder() < encoder) {
            j++;
        }

        if(encoders[i].get_encoder() == encoder) {
            std::cerr << "error: encoder " << encoder << " has already been set" << std::endl;
            error = true;
        }
    }

    if(!error) {
        std::vector<Sensor> temp(encoders_active + 1);

        std::copy( encoders.begin() + 0, encoders.begin() + 0 + encoders.size(), temp.begin());

        if(j < encoders_active) {
            std::copy( encoders.begin() + j, encoders.begin() + j+1 + encoders.size() - j, temp.begin());
        }

        temp[j] = Sensor(encoder, rotation, offset, resolution, port);
        encoder_assignment(encoder, port);

        this->encoders = temp;

        encoders_active++;
    }
}

/**
 * Add an analog sensor to platform
 *
 * @param    pin the analog pin on haply board to be used for sensor input (Ex: A0)
 */
void Device::add_analog_sensor(std::string pin)
{
    // set sensor to be size zero
    bool error = false;

    char port = pin.at(0);
    std::string number = pin.substr(1);

    int value = atoi(number.c_str());
    value += 54;

    for(int i = 0; i < sensors_active; i++) {
        if(value == sensors[i].get_port()) {
            std::cerr << "error: Analog pin: A" << (value - 54) << " has already been set" << std::endl;
            error = true;
        }
    }

    if(port != 'A' || value < 54 || value > 65) {
        std::cerr << "error: outside analog pin range" << std::endl;
        error = true;
    }

    if(!error) {
        std::vector<Sensor> temp;
        std::copy( sensors.begin(), sensors.end(), temp.begin());
        temp[sensors_active].set_port(value);
        sensors = temp;
        sensors_active++;
    }
}

/**
 * Add a PWM output pin to the platform
 *
 * @param		pin the pin on the haply board to use as the PWM output pin
 */
void Device::add_pwm_pin(int pin)
{
    bool error = false;

    for(int i = 0; i < pwms_active; i++) {
        if(pin == pwms[i].get_pin()) {
            std::cerr << "error: PWM pin: " << pin << " has already been set" << std::endl;
            error = true;
        }
    }

    if(pin < 0 || pin > 13) {
        std::cerr << "error: outside PWM pin range" << std::endl;
        error = true;
    }

    if(pin == 0 || pin == 1) {
        std::cerr << "warning: 0 and 1 are not PWM pins on Haply M3 or Haply original" << std::endl;
    }


    if(!error) {
        std::vector<Pwm> temp;
        std::copy( pwms.begin(), pwms.end(), temp.begin());

        temp[pwms_active].set_pin(pin);
        pwms = temp;
        pwms_active++;
    }
}

/**
 * Set the device mechanism that is to be used
 *
 * @param    mechanism new Mechanisms for use
 */
void Device::set_mechanism(Mechanisms *mechanism)
{
    this->mechanism = mechanism;
}

/**
 * Gathers all encoder, sensor, pwm setup information of all encoders, sensors, and pwm pins that are
 * initialized and sequentialy formats the data based on specified sensor index positions to send over
 * serial port interface for hardware device initialization
 */
void Device::device_set_parameters()
{

    communication_type = 1;

    int control;

    std::vector<float> encoder_parameters;

    std::vector<byte> encoder_params;
    std::vector<byte> motor_params;
    std::vector<byte> sensor_params;
    std::vector<byte> pwm_params;

    if(encoders_active > 0) {
        encoder_params.resize(encoders_active + 1);
        control = 0;

        for(int i = 0; i < encoders.size(); i++) {
            if(encoders[i].get_encoder() != (i+1)) {
                std::cerr << "warning, improper encoder indexing" << std::endl;
                encoders[i].set_encoder(i+1);
                encoder_positions[encoders[i].get_port() - 1] = (byte)encoders[i].get_encoder();
            }
        }

        for(int i = 0; i < encoder_positions.size(); i++) {
            control = control >> 1;

            if(encoder_positions[i] > 0) {
                control = control | 0x0008;
            }
        }

        encoder_params[0] = (byte)control;

        encoder_parameters.resize(2*encoders_active);

        int j = 0;
        for(int i = 0; i < encoder_positions.size(); i++) {
            if(encoder_positions[i] > 0) {
                encoder_parameters[2*j] = encoders[encoder_positions[i]-1].get_offset();
                encoder_parameters[2*j+1] = encoders[encoder_positions[i]-1].get_resolution();
                j++;
                encoder_params[j] = (byte)encoders[encoder_positions[i]-1].get_direction();
            }
        }
    } else {
        encoder_params.resize(1);
        encoder_params[0] = 0;
    }


    if(actuators_active > 0) {
        motor_params.resize(actuators_active + 1);
        control = 0;

        for(int i = 0; i < motors.size(); i++) {
            if(motors[i].get_actuator() != (i+1)) {
                std::cerr << "warning, improper actuator indexing" << std::endl;
                motors[i].set_actuator(i+1);
                actuator_positions[motors[i].get_port() - 1] = (byte)motors[i].get_actuator();
            }
        }

        for(int i = 0; i < actuator_positions.size(); i++) {
            control = control >> 1;

            if(actuator_positions[i] > 0) {
                control = control | 0x0008;
            }
        }

        motor_params[0] = (byte)control;

        int j = 1;
        for(int i = 0; i < actuator_positions.size(); i++) {
            if(actuator_positions[i] > 0) {
                motor_params[j] = (byte)motors[actuator_positions[i]-1].get_direction();
                j++;
            }
        }
    } else {
        motor_params.resize(1);
        motor_params[0] = 0;
    }


    if(sensors_active > 0) {
        sensor_params.resize(sensors_active + 1);
        sensor_params[0] = (byte)sensors_active;

        for(int i = 0; i < sensors_active; i++) {
            sensor_params[i+1] = (byte)sensors[i].get_port();
        }

        //Arrays.sort(sensor_params);
        std::sort(sensor_params.begin(),sensor_params.end());

        for(int i = 0; i < sensors_active; i++) {
            sensors[i].set_port(sensor_params[i+1]);
        }

    } else {
        sensor_params.resize(1);
        sensor_params[0] = 0;
    }


    if(pwms_active > 0) {
        std::vector<byte> temp(pwms_active);

        pwm_params.resize(pwms_active + 1);
        pwm_params[0] = (byte)pwms_active;


        for(int i = 0; i < pwms_active; i++) {
            temp[i] = (byte)pwms[i].get_pin();
        }

        //Arrays.sort(temp);
        std::sort(temp.begin(),temp.end());

        for(int i = 0; i < pwms_active; i++) {
            pwms[i].set_pin(temp[i]);
            pwm_params[i+1] = (byte)pwms[i].get_pin();
        }

    } else {
        pwm_params.resize(1);
        pwm_params[0] = 0;
    }

    std::vector<byte> encMtrSenPwm;

    encMtrSenPwm.resize(motor_params.size()  + encoder_params.size() + sensor_params.size() + pwm_params.size());

    std::copy( motor_params.begin(), motor_params.end(), encMtrSenPwm.begin());

    std::copy( encoder_params.begin(), encoder_params.end(), encMtrSenPwm.begin()+motor_params.size());

    std::copy( sensor_params.begin(), sensor_params.end(), encMtrSenPwm.begin()+motor_params.size()+encoder_params.size());

    std::copy( pwm_params.begin(), pwm_params.end(), encMtrSenPwm.begin()+motor_params.size()+encoder_params.size()+sensor_params.size());

    device_link->transmit(communication_type, device_id, encMtrSenPwm, encoder_parameters);
}

/**
 * assigns actuator positions based on actuator port
 */
void Device::actuator_assignment(int actuator, int port)
{
    if(actuator_positions[port - 1] > 0) {
        std::cerr << "warning, double check actuator port usage" << std::endl;
    }

    this->actuator_positions[port - 1] = (byte) actuator;
}

/**
 * assigns encoder positions based on actuator port
 */
void Device::encoder_assignment(int encoder, int port)
{

    if(encoder_positions[port - 1] > 0) {
        std::cerr << "warning, double check encoder port usage" << std::endl;
    }

    this->encoder_positions[port - 1] = (byte) encoder;
}

// device communication functions
/**
 * Receives angle position and sensor inforamation from the serial port interface and updates each indexed encoder
 * sensor to their respective received angle and any analog sensor that may be setup
 */
void Device::device_read_data()
{
    communication_type = 2;
    int data_count = 0;

    std::vector<float> device_data = device_link->receive(communication_type, device_id, sensors_active + encoders_active);

    for(int i = 0; i < sensors_active; i++) {
        sensors[i].set_value(device_data[data_count]);
        data_count++;
    }

    for(int i = 0; i < encoder_positions.size(); i++) {
        if(encoder_positions[i] > 0) {
            encoders[encoder_positions[i]-1].set_value(device_data[data_count]);
            data_count++;
        }
    }
}

/**
 * Requests data from the hardware based on the initialized setup. function also sends a torque output
 * command of zero torque for each actuator in use
 */
void Device::device_read_request()
{
    communication_type = 2;
    std::vector<byte> pulses(pwms_active);
    std::vector<float> encoder_request(actuators_active);

    for(int i = 0; i < pwms.size(); i++) {
        pulses[i] = (byte)pwms[i].get_value();
    }

    // think about this more encoder is detached from actuators
    int j = 0;
    for(int i = 0; i < actuator_positions.size(); i++) {
        if(actuator_positions[i] > 0) {
            encoder_request[j] = 0;
            j++;
        }
    }

    device_link->transmit(communication_type, device_id, pulses, encoder_request);
}

/**
 * Transmits specific torques that has been calculated and stored for each actuator over the serial
 * port interface, also transmits specified pwm outputs on pwm pins
 */
void Device::device_write_torques()
{
    communication_type = 2;
    std::vector<byte> pulses(pwms_active,0);
    std::vector<float> device_torques(actuators_active);

    for(int i = 0; i < pwms.size(); i++) {
        pulses[i] = (byte)pwms[i].get_value();
    }

    int j = 0;
    for(int i = 0; i < actuator_positions.size(); i++) {
        if(actuator_positions[i] > 0) {
            device_torques[j] = motors[actuator_positions[i]-1].get_torque();
            j++;
        }
    }

    device_link->transmit(communication_type, device_id, pulses, device_torques);
}

/**
 * Set pulse of specified PWM pin
 */
void Device::set_pwm_pulse(int pin, float pulse)
{

    for(int i = 0; i < pwms.size(); i++) {
        if(pwms[i].get_pin() == pin) {
            pwms[i].set_pulse(pulse);
        }
    }
}


/**
 * Gets percent PWM pulse value of specified pin
 */
float Device::get_pwm_pulse(int pin)
{

    float pulse = 0;

    for(int i = 0; i < pwms.size(); i++) {
        if(pwms[i].get_pin() == pin) {
            pulse = pwms[i].get_pulse();
        }
    }

    return pulse;
}

/**
 * Gathers current state of angles information from encoder objects
 *
 * @returns    most recent angles information from encoder objects
 */
std::vector<float> Device::get_device_angles()
{
    std::vector<float> angles(encoders_active);

    for(int i = 0; i < encoders_active; i++) {
        angles[i] = encoders[i].get_value();
    }

    return angles;
}


/**
 * Gathers current data from sensor objects
 *
 * @returns    most recent analog sensor information from sensor objects
 */
std::vector<float> Device::get_sensor_data()
{
    std::vector<float> data(sensors_active);

    int j = 0;
    for(int i = 0; i < sensors_active; i++) {
        data[i] = sensors[i].get_value();
    }

    return data;
}

/**
 * Performs physics calculations based on the given angle values
 *
 * @param      angles angles to be used for physics position calculation
 * @returns    end-effector coordinate position
 */
std::vector<float> Device::get_device_position(std::vector<float> angles)
{

    this->mechanism->forwardKinematics(angles);
    std::vector<float> end_effector_position = this->mechanism->get_coordinate();

    return end_effector_position;
}

/**
 * Calculates the needed output torques based on forces input and updates each initialized
 * actuator respectively
 *
 * @param     forces forces that need to be generated
 * @returns   torques that need to be outputted to the physical device
 */
std::vector<float> Device::set_device_torques(std::vector<float> forces)
{
    this->mechanism->torqueCalculation(forces);
    std::vector<float> torques = this->mechanism->get_torque();
    for (int i = 0; i < actuators_active; i++) {
        this->motors[i].set_torque(torques[i]);
    }
    return torques;
}
