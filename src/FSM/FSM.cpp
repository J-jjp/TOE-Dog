/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/FSM.h"
#include <iostream>

FSM::FSM(CtrlComponents *ctrlComp)
    :_ctrlComp(ctrlComp){

    _stateList.invalid = nullptr;
    _stateList.passive = new State_Passive(_ctrlComp);
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);
    _stateList.freeStand = new State_FreeStand(_ctrlComp);
    _stateList.rl = new State_Rl(_ctrlComp);
    initialize();
}

FSM::~FSM(){
    _stateList.deletePtr();
}

void FSM::initialize(){
    _currentState = _stateList.passive;
    _currentState -> enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;
}

void FSM::run(){
    _startTime = getSystemTime();//获取当前时间
    _ctrlComp->sendRecv();

   //在这里下发控制命令-----------------------------------------------------------------


    // _ctrlComp->runWaveGen();  //计算步态参数
    // _ctrlComp->estimator->run(); //估计器迭代一次
    // if(!checkSafty()){  //进行安全检测 如果当前不安全  就设置为阻尼模式 让机器人趴下
    //     _ctrlComp->ioInter->setPassive();
    // }
    // std::cout<<"FSM";
    // std::cout<<"imu22:";
    // for (size_t i = 0; i < 3; i++)
    // {

    //     std::cout<<_ctrlComp->lowState->imu.gyroscope[i] *obs_scales_ang_vel<<" ";
    // }
    std::cout<<std::endl;
    if(_mode == FSMMode::NORMAL){
        std::cout<<"current"<<_currentState->_stateNameString<<std::endl;
        _currentState->run();
        if (_ctrlComp->lowState->userCmd==UserCommand::PASS)
        {
            std::cout<<"FSMPASS"<<std::endl;
        }
        _nextStateName = _currentState->checkChange();
        if(_nextStateName != _currentState->_stateName){
            _mode = FSMMode::CHANGE;
            _nextState = getNextState(_nextStateName);
            std::cout << "Switched from " << _currentState->_stateNameString
                      << " to " << _nextState->_stateNameString << std::endl;
        }
    }
    else if(_mode == FSMMode::CHANGE){
        std::cout<<"next:";
        _currentState->exit();
        _currentState = _nextState;
        _currentState->enter();
        _mode = FSMMode::NORMAL;
        _currentState->run();
    }

    //在这里更新状态-----------------------------------------------------------------
    _ctrlComp->send();

    absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));//*1000 1000是为了转为微秒      2.5ms 保证400hz


}
//
FSMState* FSM::getNextState(FSMStateName stateName){
    switch (stateName)
    {
    case FSMStateName::INVALID:
        return _stateList.invalid;
        break;
    case FSMStateName::PASSIVE:
        return _stateList.passive;
        break;
    case FSMStateName::FIXEDSTAND:
        return _stateList.fixedStand;
        break;
    case FSMStateName::FREESTAND:
        return _stateList.freeStand;
        break;
    case FSMStateName::Rl:
        return _stateList.rl;
        break;
    default:
        return _stateList.invalid;
        break;
    }
}
/*
* getRotMat()函数会获得 基于IMU读取到表征躯干姿态的旋转矩阵
* 该旋转矩阵的每一列表示对应的坐标轴在世界坐标系下的投影
*
* */
// bool FSM::checkSafty(){
//     // The angle with z axis less than 60 degree
//     //如果机器人本体Z轴与世界坐标系Z轴夹角大于60度，则视为不安全
//     if(_ctrlComp->lowState->getRotMat()(2,2) < 0.5 ){
//         return false;
//     }else{
//         return true;
//     }
// }
//该保护策略已经默认添加于整个工程的状态机运行状态下，如果用户需要验证一些极端的运动(如：前空翻、后空翻、拜年等)，需要注意此策略的影响

