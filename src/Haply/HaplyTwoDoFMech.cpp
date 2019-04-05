/**
 **********************************************************************************************************************
 * @file       HaplyTwoDoFMech.cpp
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

#include "Haply/HaplyTwoDoFMech.h"

#include<math.h>

using namespace Haply;

HaplyTwoDoFMech::HaplyTwoDoFMech() : Mechanisms()
{
    pi = 3.14159265359f;
    gain = 0.1f;
    this->l = 0.05f; //m
    this->L = 0.07f;
    this->d = 0.02f;
}

void HaplyTwoDoFMech::forwardKinematics(std::vector<float> angles)
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

void HaplyTwoDoFMech::torqueCalculation(std::vector<float> force)
{
    f_x = force[0];
    f_y = force[1];

    tau1 = J11*f_x + J12*f_y;
    tau2 = J21*f_x + J22*f_y;

    tau1 = -tau1*gain;
    tau2 =  tau2*gain;

}

void HaplyTwoDoFMech::forceCalculation()
{
}

void HaplyTwoDoFMech::positionControl()
{
}

void HaplyTwoDoFMech::inverseKinematics()
{
}

void HaplyTwoDoFMech::set_mechanism_parameters(std::vector<float> parameters)
{
    l = parameters[0];
    L = parameters[1];
    d = parameters[2];
}

std::vector<float> HaplyTwoDoFMech::get_coordinate()
{
    std::vector<float>temp;
    temp.reserve(2);
    temp.insert(temp.begin(),x_E);
    temp.insert(temp.begin()+1,y_E);
    return temp;
}

std::vector<float> HaplyTwoDoFMech::get_torque()
{
    std::vector<float>temp;
    temp.reserve(2);
    temp.insert(temp.begin(),tau1);
    temp.insert(temp.begin()+1,tau2);
    return temp;
}

std::vector<float> HaplyTwoDoFMech::get_angle()
{
    std::vector<float>temp;
    temp.reserve(2);
    temp.insert(temp.begin(),th1);
    temp.insert(temp.begin()+1,th2);
    return temp;
}
