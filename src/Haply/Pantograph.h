/**
 **********************************************************************************************************************
 * @file       Pantograph.h
 * @author     Steve Ding, Colin Gallacher, Christian Frisson
 * @version    V2.0.0
 * @date       23-April-2019
 * @brief      Mechanism extension Pantograph example
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */

#ifndef __HaplyPantograph__
#define __HaplyPantograph__

#include "Haply/Mechanisms.h"

namespace Haply
{

class Pantograph: public Mechanisms
{

private:

    float l, L, d;

    float th1, th2;
    float tau1, tau2;
    float f_x, f_y;
    float x_E, y_E;

    float pi = 3.14159265359f;
    float J11, J12, J21, J22;
    float gain = 0.1f;

public:
    Pantograph();

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


}; // class Pantograph

} // namespace Haply

#endif // __HaplyPantograph__