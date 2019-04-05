/**
 **********************************************************************************************************************
 * @file       HaplyFourDoFMech.cpp
 * @author     Colin Gallacher, Steven Ding, Christian Frisson
 * @version    V0.1.0
 * @date       14-December-2017
 * @brief      Haply 4-DOF Mechanism (TODO)
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */

#include "Haply/HaplyFourDoFMech.h"

using namespace Haply;

HaplyFourDoFMech::HaplyFourDoFMech() : Mechanisms()
{
}


void HaplyFourDoFMech::forwardKinematics(std::vector<float> angles)
{
}

void HaplyFourDoFMech::torqueCalculation(std::vector<float> force)
{
}

void HaplyFourDoFMech::forceCalculation()
{
}

void HaplyFourDoFMech::positionControl()
{
}

void HaplyFourDoFMech::inverseKinematics()
{
}

void HaplyFourDoFMech::set_mechanism_parameters(std::vector<float> parameters)
{
}

std::vector<float> HaplyFourDoFMech::get_coordinate()
{
    std::vector<float>temp;
    //temp.reserve(2);
    //temp.insert(temp.begin(), ...);
    //temp.insert(temp.begin(), ...);
    return temp;
}


std::vector<float> HaplyFourDoFMech::get_torque()
{
    std::vector<float>temp;
    //temp.reserve(2);
    //temp.insert(temp.begin(), ...);
    //temp.insert(temp.begin(), ...);
    return temp;
}


std::vector<float> HaplyFourDoFMech::get_angle()
{
    std::vector<float>temp;
    //temp.reserve(2);
    //temp.insert(temp.begin(), ...);
    //temp.insert(temp.begin(), ...);
    return temp;
}

