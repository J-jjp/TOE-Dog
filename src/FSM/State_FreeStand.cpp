/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_FreeStand.h"
State_FreeStand::State_FreeStand(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::FREESTAND, "free stand"){
    _rowMax = 10 * M_PI / 180;//20弧度
    _rowMin = -_rowMax;
    _pitchMax = 15 * M_PI / 180;//
    _pitchMin = -_pitchMax;
    _yawMax = 15 * M_PI / 180;//20弧度
    _yawMin = -_yawMax;
    _heightMax = 0.04;//4
    _heightMin = -_heightMax;
}

void State_FreeStand::enter(){
    for(int i=0; i<4; i++){
        if(_ctrlComp->ctrlPlatform == CtrlPlatform::Mujoco){
            _lowCmd->setSimStanceGain(i);
        }
        else if(_ctrlComp->ctrlPlatform == CtrlPlatform::REALROBOT){
            _lowCmd->setRealStanceGain(i);
        }
    }
    _initVecOX = _ctrlComp->robotModel->getX(*_lowState);//0号脚的坐标
    _initVecXP = _ctrlComp->robotModel->getVecXP(*_lowState);//四个脚的坐标以0号脚为原点
    
    _ctrlComp->setAllStance();
    _ctrlComp->ioInter->zeroCmdPanel();
    Vec34 vecOP;
    _userValue = _lowState->userValue;
    std::cout<<"_userValue.lx"<<_userValue.lx;
    vecOP = _calcOP( invNormalize(_userValue.lx, _rowMin, _rowMax),
                     invNormalize(_userValue.ly, _pitchMin, _pitchMax),
                    -invNormalize(_userValue.rx, _yawMin, _yawMax),
                     invNormalize(_userValue.ry, _heightMin, _heightMax) );
    Vec12 q = _ctrlComp->robotModel->getQ(vecOP, FrameType::BODY);
    for (size_t i = 0; i < 12; i++)
    {
        _defos[i]=_lowCmd->motorCmd[i].q-q[i];
    }
}

void State_FreeStand::run(){
    Vec34 vecOP;
    speed_limit();
    vecOP = _calcOP( invNormalize(_userValue.lx, _rowMin, _rowMax),
                     invNormalize(_userValue.ly, _pitchMin, _pitchMax),
                    -invNormalize(_userValue.rx, _yawMin, _yawMax),
                     invNormalize(_userValue.ry, _heightMin, _heightMax) );
    _calcCmd(vecOP);
    Vec3 eu_ang;
    eu_ang = quaternion_to_euler_array(_lowState->imu.getQuat());
    std::cout<<"olx:"<<eu_ang[0]<<"\ty:"<<eu_ang[1]<<"\tz:"<<eu_ang[2]<<std::endl;
}

void State_FreeStand::exit(){
    _ctrlComp->ioInter->zeroCmdPanel();
}

FSMStateName State_FreeStand::checkChange(){
    if(_lowState->userCmd == UserCommand::FIXED){
        return FSMStateName::FIXEDSTAND;
    }
    else if(_lowState->userCmd == UserCommand::PASS){
        return FSMStateName::PASSIVE;
    }
    else{
        return FSMStateName::FREESTAND;
    }
}

Vec34 State_FreeStand::_calcOP(float row, float pitch, float yaw, float height){
    Vec3 vecXO = -_initVecOX;//0号脚的坐标
    vecXO(2) += height;

    RotMat rotM = rpyToRotMat(row, pitch, yaw);//旋转矩阵

    HomoMat Tsb = homoMatrix(vecXO, rotM);//合为齐次矩阵
    HomoMat Tbs = homoMatrixInverse(Tsb);

    Vec4 tempVec4;
    Vec34 vecOP;
    for(int i(0); i<4; ++i){
        tempVec4 = Tbs * homoVec(_initVecXP.col(i));
        vecOP.col(i) = noHomoVec(tempVec4);
    }

    return vecOP;
}

void State_FreeStand::_calcCmd(Vec34 vecOP){
    Vec12 q = _ctrlComp->robotModel->getQ(vecOP, FrameType::BODY);
    // for (size_t i = 0; i < 12; i++)
    // {
    //     std::cout<<"\tsend第"<<i<<"条"<<q[i];
    // }
    for (size_t i = 0; i < 12; i++)
    {
        q[i] += _defos[i];
    }

    _lowCmd->setQ(q);
}
void State_FreeStand::speed_limit(){

    if (std::abs(_lowState->userValue.lx - _userValue.lx) > 0.01) {
        // 根据差值的正负决定增加还是减少_userValue.lx
        _userValue.lx += (_lowState->userValue.lx > _userValue.lx) ? 0.01 : -0.01;
    } else {
        // 如果差值不大于0.1，直接设置为_lowState->userValue.lx的值
        _userValue.lx = _lowState->userValue.lx;
    }
    if (std::abs(_lowState->userValue.ly - _userValue.ly) > 0.01) {
        // 根据差值的正负决定增加还是减少_userValue.lx
        _userValue.ly += (_lowState->userValue.ly > _userValue.ly) ? 0.01 : -0.01;
    } else {
        // 如果差值不大于0.1，直接设置为_lowState->userValue.lx的值
        _userValue.ly = _lowState->userValue.ly;
    }
    if (std::abs(_lowState->userValue.rx - _userValue.rx) > 0.01) {
        // 根据差值的正负决定增加还是减少_userValue.lx
        _userValue.rx += (_lowState->userValue.rx > _userValue.rx) ? 0.01 : -0.01;
    } else {
        // 如果差值不大于0.1，直接设置为_lowState->userValue.lx的值
        _userValue.rx = _lowState->userValue.rx;
    }
    if (std::abs(_lowState->userValue.ry - _userValue.ry) > 0.01) {
        // 根据差值的正负决定增加还是减少_userValue.lx
        _userValue.ry += (_lowState->userValue.ry > _userValue.ry) ? 0.01 : -0.01;
    } else {
        // 如果差值不大于0.1，直接设置为_lowState->userValue.lx的值
        _userValue.ry = _lowState->userValue.ry;
    }
}
//  send第0条0.110013       send第1条0.790046       send第2条-1.57704       send第3条-0.110013      send第4条0.790049       
//  send第5条-1.57704       send第6条0.0401595     send第7条1.03583 send第8条-1.5356        send第9条-0.0401596\
//   send第10条1.03583       send第11条-1.5356

// send第0条0.0401851      send第1条0.835998       send第2条-1.56248       send第3条-0.0406991     send第4条0.836216       
// send第5条-1.56248       send第6条-0.00186835   send第7条1.00875 send第8条-1.5444        send第9条0.000866268   
//  send第10条1.00812       send第11条-1.5445
Vec3 State_FreeStand::quaternion_to_euler_array(Vec4 quat){
    double x = quat[1];
    double y = quat[2];
    double z = quat[3];
    double w = quat[0];
    double t0, t1, t2, t3,t4;
    double roll_x, pitch_y, yaw_z;
    t0 = +2.0 * (w * x + y * z);
    t1 = +1.0 - 2.0 * (x * x + y * y);
    roll_x = std::atan2(t0, t1);
    
    t2 = +2.0 * (w * y - z * x);
    t2 = std::max(-1.0, std::min(t2, 1.0));
    pitch_y =  std::asin(t2);
    
    t3 = +2.0 * (w * z + x * y);
    t4 = +1.0 - 2.0 * (y * y + z * z);
    yaw_z =  std::atan2(t3, t4);
    return {roll_x, pitch_y, yaw_z};
}