/**
 **********************************************************************************************************************
 * @file       Pantograph.cpp
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

#include "Haply/Pantograph.h"

#include <math.h>

using namespace Haply;
using namespace std;

Pantograph::Pantograph()
    : l(0.05), L(0.07), d(0.02),
      th1(0.0), th2(0.0),
      tau1(0.0), tau2(0.0),
      f_x(0.0), f_y(0.0),
      x_E(0.0), y_E(0.0),
      pi(3.14159265359),
      J11(0.0), J12(0.0), J21(0.0), J22(0.0),
      gain(0.1)
{

}

/**
 * Performs the forward kinematics physics calculation of a specific physical mechanism
 *
 * @param    angles angular inpujts of physical mechanisms (array element length based
 *           on the degree of freedom of the mechanism in question)
 */
void Pantograph::forwardKinematics(std::vector<float> angles)
{
    th1 = pi/180*angles[0];
    th2 = pi/180*angles[1];

    // Forward Kinematics
    float c1 = (float)cos(th1);
    float c2 = (float)cos(th2);
    float s1 = (float)sin(th1);
    float s2 = (float)sin(th2);

    float xA = l*c1;
    float yA = l*s1;
    float xB = d+l*c2;
    float yB = l*s2;
    float R = (float)pow(xA,2) + (float)pow(yA,2);
    float S = (float)pow(xB,2) + (float)pow(yB,2);
    float M = (yA-yB)/(xB-xA);
    float N = (float)0.5*(S-R)/(xB-xA);
    float a = (float)pow(M,2)+1;
    float b = 2*(M*N-M*xA-yA);
    float c = (float)pow(N,2)-2*N*xA+R-(float)pow(L,2);
    float Delta = (float)pow(b,2)-4*a*c;

    y_E = (-b+(float)sqrt(Delta))/(2*a);
    x_E = M*y_E+N;

    float phi1 = (float)acos((x_E-l*c1)/L);
    float phi2 = (float)acos((x_E-d-l*c2)/L);
    float s21 = (float)sin(phi2-phi1);
    float s12 = (float)sin(th1-phi2);
    float s22 = (float)sin(th2-phi2);
    J11 = -(s1*s21 + (float)sin(phi1)*s12)/s21;
    J12 = (c1*s21 + (float)cos(phi1)*s12)/s21;
    J21 = (float)sin(phi1)*s22/s21;
    J22 = -(float)cos(phi1)*s22/s21;
}

/**
 * Performs torque calculations that actuators need to output
 *
 * @param    force force values calculated from physics simulation that needs to be counteracted
 *
 */
void Pantograph::torqueCalculation(std::vector<float> forces)
{
    f_x = forces[0];
    f_y = forces[1];

    tau1 = J11*f_x + J12*f_y;
    tau2 = J21*f_x + J22*f_y;

    tau1 = tau1*gain;
    tau2 = tau2*gain;
}

/**
 * Performs force calculations
 */
void Pantograph::forceCalculation()
{
}


/**
 * Performs calculations for position control
 */
void Pantograph::positionControl()
{
}


/**
 * Performs inverse kinematics calculations
 */
void Pantograph::inverseKinematics()
{
}

/**
 * Initializes or changes mechanisms parameters
 *
 * @param    parameters mechanism parameters
 */
void Pantograph::set_mechanism_parameters(std::vector<float> parameters)
{
    this->l = parameters[0];
    this->L = parameters[1];
    this->d = parameters[2];
}

/**
 * Sets and updates sensor data that may be used by the mechanism
 *
 * @param    data sensor data from sensors attached to Haply board
 */
void Pantograph::set_sensor_data(std::vector<float> data)
{
}


/**
 * @return   end-effector coordinate position
 */
std::vector<float> Pantograph::get_coordinate()
{
    std::vector<float> temp;
    temp.push_back(x_E);
    temp.push_back(y_E);
    return temp;
}

/**
 * @return   torque values from physics calculations
 */
std::vector<float> Pantograph::get_torque()
{
    std::vector<float> temp;
    temp.push_back(tau1);
    temp.push_back(tau2);
    return temp;
}

/**
 * @return   angle values from physics calculations
 */
std::vector<float> Pantograph::get_angle()
{
    std::vector<float> temp;
    temp.push_back(th1);
    temp.push_back(th2);
    return temp;
}
