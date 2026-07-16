#include "dynamixel_controller.hpp"

Dxl_Controller::Dxl_Controller(Dxl* dxlPtr)
    : dxlPtr(dxlPtr)
{
}

VectorXd Dxl_Controller::GetJointTheta()
{
    th_cont = dxlPtr->GetThetaAct();
    return th_cont;
}

VectorXd Dxl_Controller::GetThetaDot()
{
    th_dot_cont = dxlPtr->GetThetaDot();
    return th_dot_cont;
}

VectorXd Dxl_Controller::GetThetaDotMAF()
{
    th_dot_cont = dxlPtr->GetThetaDotEstimated();
    MAF.topRows(Window_Size - 1) = MAF.bottomRows(Window_Size - 1);
    MAF.bottomRows(1) = th_dot_cont.transpose();
    th_dot_MovAvgFilterd = MAF.colwise().mean();
    return th_dot_MovAvgFilterd;
}

VectorXd Dxl_Controller::GetTorque()
{
    return VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
}

void Dxl_Controller::SetTorque(VectorXd tau)
{
    torque_cont = tau;
    dxlPtr->SetTorqueRef(torque_cont);
}

void Dxl_Controller::SetPosition(VectorXd theta)
{
    th_cont = theta;
    dxlPtr->SetThetaRef(th_cont);
}
