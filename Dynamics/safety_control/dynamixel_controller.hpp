#ifndef DYNAMIXEL_CONTROLLER_H
#define DYNAMIXEL_CONTROLLER_H

#include "dynamixel.hpp"

#define Window_Size 2

using Eigen::MatrixXd;

class Dxl_Controller
{
public:
    explicit Dxl_Controller(Dxl* dxlPtr);

    virtual VectorXd GetJointTheta();
    virtual VectorXd GetThetaDot();
    virtual VectorXd GetThetaDotMAF();
    virtual VectorXd GetTorque();
    virtual void SetTorque(VectorXd tau);
    virtual void SetPosition(VectorXd theta);

    Dxl* dxlPtr;
    VectorXd th_cont = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
    VectorXd th_dot_cont = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
    VectorXd th_dot_MovAvgFilterd =
        VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
    MatrixXd MAF = MatrixXd::Zero(Window_Size, NUMBER_OF_DYNAMIXELS);
    VectorXd torque_cont = VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
};

#endif
