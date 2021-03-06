/**
 **********************************************************************************************************************
 * @file       Mechanisms.h
 * @author     Colin Gallacher, Steven Ding, Christian Frisson
 * @version    V2.0.0
 * @date       17-April-2019
 * @brief      Mechanisms class designed for use as a template.
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */

#ifndef __HaplyMechanisms__
#define __HaplyMechanisms__

#include <vector>

namespace Haply
{

class Mechanisms
{

    /**
     * Performs the forward kinematics physics calculation of a specific physical mechanism
     *
     * @param    angles angular inpujts of physical mechanisms (array element length based
     *           on the degree of freedom of the mechanism in question)
     */
public:
    virtual void forwardKinematics(std::vector<float> angles) = 0;


    /**
     * Performs torque calculations that actuators need to output
     *
     * @param    force force values calculated from physics simulation that needs to be counteracted
     *
     */
public:
    virtual void torqueCalculation(std::vector<float> forces) = 0;


    /**
     * Performs force calculations
     */
public:
    virtual void forceCalculation() = 0;


    /**
     * Performs calculations for position control
     */
public:
    virtual void positionControl() = 0;


    /**
     * Performs inverse kinematics calculations
     */
public:
    virtual void inverseKinematics() = 0;


    /**
     * Initializes or changes mechanisms parameters
     *
     * @param    parameters mechanism parameters
     */
public:
    virtual void set_mechanism_parameters(std::vector<float> parameters) = 0;


    /**
     * Sets and updates sensor data that may be used by the mechanism
     *
     * @param    data sensor data from sensors attached to Haply board
     */
public:
    virtual void set_sensor_data(std::vector<float> data) = 0;


    /**
     * @return   end-effector coordinate position
     */
public:
    virtual std::vector<float> get_coordinate() = 0;

    /**
     * @return   torque values from physics calculations
     */
public:
    virtual std::vector<float> get_torque() = 0;


    /**
     * @return   angle values from physics calculations
     */
public:
    virtual std::vector<float> get_angle() = 0;

}; // class Mechanisms

} // namespace Haply

#endif // __HaplyMechanisms__
