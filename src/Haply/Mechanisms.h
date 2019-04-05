/**
 **********************************************************************************************************************
 * @file       Mechanisms.h
 * @author     Colin Gallacher, Steven Ding, Christian Frisson
 * @version    V0.1.0
 * @date       01-March-2017
 * @brief      Mechanisms class designed for use as a template. Current classes which extends the Mechanisms
 *             class includes:
 *                - HapticPaddle
 *                - HaplyOneDoFMech
 *                - HaplyTwoDoFMech
 *                - HaplyThreeDoFMech
 *                - HaplyFourDoFMech
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
     * @param    force force values calculated from physics simulation that needs to be conteracted
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

};

}

#endif // __HaplyMechanisms__
