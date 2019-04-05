/**
 **********************************************************************************************************************
 * @file       HapticPaddleMech.h
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

#ifndef __HaplyHapticPaddleMech__
#define __HaplyHapticPaddleMech__

#include "Haply/Mechanisms.h"

namespace Haply
{

class HapticPaddleMech: public Mechanisms
{

private:
    float     rh; // length of handle in m
    float     angle;
    float     torque;
    float     xh;

public:
    HapticPaddleMech();

    virtual void forwardKinematics(std::vector<float> angles);
    virtual void torqueCalculation(std::vector<float> force);
    virtual void forceCalculation();
    virtual void positionControl();
    virtual void inverseKinematics();
    virtual void set_mechanism_parameters(std::vector<float> parameters);
    virtual std::vector<float> get_coordinate();
    virtual std::vector<float> get_torque();
    virtual std::vector<float> get_angle();
};

}

#endif // __HaplyHapticPaddleMech__
