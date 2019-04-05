/**
 **********************************************************************************************************************
 * @file       HapticPaddleMech.cpp
 * @author     Colin Gallacher, Steven Ding, Christian Frisson
 * @version    V0.1.0
 * @date       14-December-2017
 * @brief      Haply Haptic Paddle Mechanism
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */

#include "Haply/HapticPaddleMech.h"

using namespace Haply;

HapticPaddleMech::HapticPaddleMech() : Mechanisms()
{
    rh       = 0.075f; // length of handle in m
    xh       = 0.0f;
}


void HapticPaddleMech::forwardKinematics(std::vector<float> angles)
{
    xh = rh*angles[0];
}


void HapticPaddleMech::torqueCalculation(std::vector<float> force)
{
}


void HapticPaddleMech::forceCalculation()
{
}


void HapticPaddleMech::positionControl()
{
}


void HapticPaddleMech::inverseKinematics()
{
}

void HapticPaddleMech::set_mechanism_parameters(std::vector<float> parameters)
{
    angle = parameters[0];
    torque = parameters[1];

}

std::vector<float> HapticPaddleMech::get_coordinate()
{
    std::vector<float> temp(1);
    temp[0] = angle;
    this->forwardKinematics(temp);

    temp[0] = xh;
    return temp;
}


std::vector<float> HapticPaddleMech::get_torque()
{
    std::vector<float> temp(1);
    temp[0] = torque;
    return temp;
}


std::vector<float> HapticPaddleMech::get_angle()
{
    std::vector<float> temp(1);
    temp[0] = angle;
    return temp;
}
