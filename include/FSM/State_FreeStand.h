/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FREESTAND_H
#define FREESTAND_H

#include "FSM/FSMState.h"

class State_FreeStand : public FSMState{
public:
    State_FreeStand(CtrlComponents *ctrlComp);
    ~State_FreeStand(){}
    void enter();
    void run();
    void exit();
    Eigen::Vector3d quat_rotate_inverse(const Eigen::Vector4d& q, const Eigen::Vector3d& v);
    FSMStateName checkChange();
    
private:
    Vec3 _initVecOX;
    Vec34 _initVecXP;
    float _rowMax, _rowMin;
    float _pitchMax, _pitchMin;
    float _yawMax, _yawMin;
    float _heightMax, _heightMin;
    float _targetPos[12] = {-0.1,0.8,-1.5 ,0.1,0.8,-1.5,-0.1,1,-1.5, 0.1,1.,-1.5};
    float _defos[12] = {0.,0.,0. ,0.,0.,0,0.,0,0, 0.,0.,0};
    Vec34 _calcOP(float row, float pitch, float yaw, float height);
    void _calcCmd(Vec34 vecOP);
    void speed_limit();
};

#endif  // FREESTAND_H