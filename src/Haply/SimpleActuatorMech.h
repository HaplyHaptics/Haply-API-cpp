/**
 **********************************************************************************************************************
 * @file       SimpleActuatorMech.h
 * @author     Steve Ding, Colin Gallacher, Christian Frisson
 * @version    V1.0.0
 * @date       23-April-2019
 * @brief      Mechanism extension simple example
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */

#ifndef __HaplySimpleActuatorMech__
#define __HaplySimpleActuatorMech__

#include "Haply/Mechanisms.h"

namespace Haply
{

class SimpleActuatorMech: public Mechanisms
{

private:
    float tau;

public:
    SimpleActuatorMech();

    /**
     * Performs the forward kinematics physics calculation of a specific physical mechanism
     *
     * @param    angles angular inpujts of physical mechanisms (array element length based
     *           on the degree of freedom of the mechanism in question)
     */
public:
    virtual void forwardKinematics(std::vector<float> angles);


    /**
     * Performs torque calculations that actuators need to output
     *
     * @param    force force values calculated from physics simulation that needs to be counteracted
     *
     */
public:
    virtual void torqueCalculation(std::vector<float> forces);


    /**
     * Performs force calculations
     */
public:
    virtual void forceCalculation();


    /**
     * Performs calculations for position control
     */
public:
    virtual void positionControl();


    /**
     * Performs inverse kinematics calculations
     */
public:
    virtual void inverseKinematics();


    /**
     * Initializes or changes mechanisms parameters
     *
     * @param    parameters mechanism parameters
     */
public:
    virtual void set_mechanism_parameters(std::vector<float> parameters);


    /**
     * Sets and updates sensor data that may be used by the mechanism
     *
     * @param    data sensor data from sensors attached to Haply board
     */
public:
    virtual void set_sensor_data(std::vector<float> data);


    /**
     * @return   end-effector coordinate position
     */
public:
    virtual std::vector<float> get_coordinate();

    /**
     * @return   torque values from physics calculations
     */
public:
    virtual std::vector<float> get_torque();


    /**
     * @return   angle values from physics calculations
     */
public:
    virtual std::vector<float> get_angle();


}; // class SimpleActuatorMech

} // namespace Haply

#endif // __HaplySimpleActuatorMech__