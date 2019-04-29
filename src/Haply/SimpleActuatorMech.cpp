/**
 **********************************************************************************************************************
 * @file       SimpleActuatorMech.cpp
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

#include "Haply/SimpleActuatorMech.h"

using namespace Haply;

SimpleActuatorMech::SimpleActuatorMech()
    :tau(0)
{

}

/**
 * Performs the forward kinematics physics calculation of a specific physical mechanism
 *
 * @param    angles angular inpujts of physical mechanisms (array element length based
 *           on the degree of freedom of the mechanism in question)
 */
void SimpleActuatorMech::forwardKinematics(std::vector<float> angles)
{
}

/**
 * Performs torque calculations that actuators need to output
 *
 * @param    force force values calculated from physics simulation that needs to be counteracted
 *
 */
void SimpleActuatorMech::torqueCalculation(std::vector<float> forces)
{
    tau = forces[0];
}

/**
 * Performs force calculations
 */
void SimpleActuatorMech::forceCalculation()
{
}


/**
 * Performs calculations for position control
 */
void SimpleActuatorMech::positionControl()
{
}


/**
 * Performs inverse kinematics calculations
 */
void SimpleActuatorMech::inverseKinematics()
{
}

/**
 * Initializes or changes mechanisms parameters
 *
 * @param    parameters mechanism parameters
 */
void SimpleActuatorMech::set_mechanism_parameters(std::vector<float> parameters)
{
}

/**
 * Sets and updates sensor data that may be used by the mechanism
 *
 * @param    data sensor data from sensors attached to Haply board
 */
void SimpleActuatorMech::set_sensor_data(std::vector<float> data)
{
}


/**
 * @return   end-effector coordinate position
 */
std::vector<float> SimpleActuatorMech::get_coordinate()
{
    std::vector<float> temp;
    temp.push_back(0);
    return temp;
}

/**
 * @return   torque values from physics calculations
 */
std::vector<float> SimpleActuatorMech::get_torque()
{
    std::vector<float> temp;
    temp.push_back(tau);
    return temp;
}

/**
 * @return   angle values from physics calculations
 */
std::vector<float> SimpleActuatorMech::get_angle()
{
    std::vector<float> temp;
    temp.push_back(0);
    return temp;
}
