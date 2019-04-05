/**
 **********************************************************************************************************************
 * @file       HaplyThreeDoFMech.cpp
 * @author     Colin Gallacher, Steven Ding, Christian Frisson
 * @version    V0.1.0
 * @date       14-December-2017
 * @brief      Haply 3-DOF Mechanism (TODO)
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */

#include "Haply/HaplyThreeDoFMech.h"

using namespace Haply;

HaplyThreeDoFMech::HaplyThreeDoFMech() : Mechanisms()
{
}


void HaplyThreeDoFMech::forwardKinematics(std::vector<float> angles)
{
}


void HaplyThreeDoFMech::torqueCalculation(std::vector<float> force)
{
}


void HaplyThreeDoFMech::forceCalculation()
{
}


void HaplyThreeDoFMech::positionControl()
{
}


void HaplyThreeDoFMech::inverseKinematics()
{
}



void HaplyThreeDoFMech::set_mechanism_parameters(std::vector<float> parameters)
{
}


std::vector<float> HaplyThreeDoFMech::get_coordinate()
{
    std::vector<float>temp;
    //temp.reserve(2);
    //temp.insert(temp.begin(), ...);
    //temp.insert(temp.begin(), ...);
    return temp;
}


std::vector<float> HaplyThreeDoFMech::get_torque()
{
    std::vector<float>temp;
    //temp.reserve(2);
    //temp.insert(temp.begin(), ...);
    //temp.insert(temp.begin(), ...);
    return temp;
}


std::vector<float> HaplyThreeDoFMech::get_angle()
{
    std::vector<float>temp;
    //temp.reserve(2);
    //temp.insert(temp.begin(), ...);
    //temp.insert(temp.begin(), ...);
    return temp;
}
