/**
 **********************************************************************************************************************
 * @file       HaplyOneDoFMech.cpp
 * @author     Colin Gallacher, Steven Ding, Christian Frisson
 * @version    V0.1.0
 * @date       14-December-2017
 * @brief      Haply 1-DOF Mechanism (TODO)
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */

#include "Haply/HaplyOneDoFMech.h"

using namespace Haply;

HaplyOneDoFMech::HaplyOneDoFMech() : Mechanisms()
{
}

void HaplyOneDoFMech::forwardKinematics(std::vector<float> angles)
{
}

void HaplyOneDoFMech::torqueCalculation(std::vector<float> force)
{
}

void HaplyOneDoFMech::forceCalculation()
{
}

void HaplyOneDoFMech::positionControl()
{
}

void HaplyOneDoFMech::inverseKinematics()
{
}

void HaplyOneDoFMech::set_mechanism_parameters(std::vector<float> parameters)
{
}

std::vector<float> HaplyOneDoFMech::get_coordinate()
{
    std::vector<float> temp;
    //temp.reserve(2);
    //temp.insert(temp.begin(), ...);
    //temp.insert(temp.begin(), ...);
    return temp;
}

std::vector<float> HaplyOneDoFMech::get_torque()
{
    std::vector<float> temp;
    //temp.reserve(2);
    //temp.insert(temp.begin(), ...);
    //temp.insert(temp.begin(), ...);
    return temp;
}

std::vector<float> HaplyOneDoFMech::get_angle()
{
    std::vector<float> temp;
    //temp.reserve(2);
    //temp.insert(temp.begin(), ...);
    //temp.insert(temp.begin(), ...);
    return temp;
}
