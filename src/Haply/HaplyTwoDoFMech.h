/**
 **********************************************************************************************************************
 * @file       HaplyTwoDoFMech.h
 * @author     Colin Gallacher, Steven Ding, Christian Frisson
 * @version    V0.1.0
 * @date       14-December-2017
 * @brief      Haply 2-DOF Mechanism
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */

#ifndef __HaplyHaplyTwoDoFMech__
#define __HaplyHaplyTwoDoFMech__

#include "Haply/Mechanisms.h"

namespace Haply
{

class HaplyTwoDoFMech: public Mechanisms
{

private:
    float 	l, L, d;
    float 	th1, th2;
    float 	tau1, tau2;
    float 	f_x, f_y;
    float 	x_E, y_E;
    float 	pi;
    float 	J11, J12, J21, J22; //Jacobian
    float   gain;

public:
    HaplyTwoDoFMech();

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

#endif // __HaplyHaplyTwoDoFMech__
