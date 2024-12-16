/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "interface/IOMujoco.h"
#include "interface/KeyBoard.h"
#include "interface/WirelessHandle.h"
#include <iostream>
#include <unistd.h>
#include <csignal>

void SimShutDown(int sig){
	ROS_INFO("SIM interface shutting down!");
	ros::shutdown();
}



// IOSIM::~IOSIM(){
//     delete cmdPanel;
// }

float IOMujoco::pd_control(float target_q,float q,float kp,float target_dq,float dq,float kd){
    float tau=(target_q - q) * kp + (target_dq - dq) * kd;
    return tau;
}

void IOMujoco::sendRecv( LowlevelCmd *cmd, LowlevelState *state){
    recvState(state);
    sendCmd(cmd,state);

    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
}

void IOMujoco::sendCmd( LowlevelCmd *lowCmd, LowlevelState *state){
    for(int i=0; i < 12; i++){
        pd_control(_lowCmd.motorCmd[i].q,state->motorState)
        _data->ctrl[i] = lowCmd->motorCmd[i].tau;
    }
}

// void IOMujoco::recvState(LowlevelState *state){
//     for(int i(0); i < 12; ++i){
//         state->motorState[i].q = _lowState.motorState[i].q;
//         state->motorState[i].dq = _lowState.motorState[i].dq;
//         state->motorState[i].ddq = _lowState.motorState[i].ddq;
//         state->motorState[i].tauEst = _lowState.motorState[i].tauEst;
//     }
//     for(int i(0); i < 3; ++i){
//         state->imu.quaternion[i] = _lowState.imu.quaternion[i];
//         state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
//         state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];
//     }
//     state->imu.quaternion[3] = _lowState.imu.quaternion[3];
// }

void IOMujoco::Robot_stateCallback(LowlevelState *state){
    for (int i = 0; i < 12; i++)
    {
        state->motorState[i].q = _data->sensordata[i];;
    }
    for (int i = 12; i < 24; i++)
    {
        _lowState.motorState[i-12].q = _data->sensordata[i];;
    }
    for (int i = 24; i < 36; i++)
    {
        _lowState.motorState[i-24].tauEst = _data->sensordata[i];;
    }
}

void IOSIM::imuCallback(const sensor_msgs::Imu & msg)
{ 
    _lowState.imu.quaternion[0] = msg.orientation.w;
    _lowState.imu.quaternion[1] = msg.orientation.x;
    _lowState.imu.quaternion[2] = msg.orientation.y;
    _lowState.imu.quaternion[3] = msg.orientation.z;

    _lowState.imu.gyroscope[0] = msg.angular_velocity.x;
    _lowState.imu.gyroscope[1] = msg.angular_velocity.y;
    _lowState.imu.gyroscope[2] = msg.angular_velocity.z;
    
    _lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
    _lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
    _lowState.imu.accelerometer[2] = msg.linear_acceleration.z;
}

void IOSIM::FRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[0].mode = msg.mode;
    _lowState.motorState[0].q = msg.q;
    _lowState.motorState[0].dq = msg.dq;
    _lowState.motorState[0].tauEst = msg.tauEst;
}

void IOSIM::FRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[1].mode = msg.mode;
    _lowState.motorState[1].q = msg.q;
    _lowState.motorState[1].dq = msg.dq;
    _lowState.motorState[1].tauEst = msg.tauEst;
}

void IOSIM::FRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[2].mode = msg.mode;
    _lowState.motorState[2].q = msg.q;
    _lowState.motorState[2].dq = msg.dq;
    _lowState.motorState[2].tauEst = msg.tauEst;
}

void IOSIM::FLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[3].mode = msg.mode;
    _lowState.motorState[3].q = msg.q;
    _lowState.motorState[3].dq = msg.dq;
    _lowState.motorState[3].tauEst = msg.tauEst;
}

void IOSIM::FLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[4].mode = msg.mode;
    _lowState.motorState[4].q = msg.q;
    _lowState.motorState[4].dq = msg.dq;
    _lowState.motorState[4].tauEst = msg.tauEst;
}

void IOSIM::FLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[5].mode = msg.mode;
    _lowState.motorState[5].q = msg.q;
    _lowState.motorState[5].dq = msg.dq;
    _lowState.motorState[5].tauEst = msg.tauEst;
}

void IOSIM::RRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[6].mode = msg.mode;
    _lowState.motorState[6].q = msg.q;
    _lowState.motorState[6].dq = msg.dq;
    _lowState.motorState[6].tauEst = msg.tauEst;
}

void IOSIM::RRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[7].mode = msg.mode;
    _lowState.motorState[7].q = msg.q;
    _lowState.motorState[7].dq = msg.dq;
    _lowState.motorState[7].tauEst = msg.tauEst;
}

void IOSIM::RRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[8].mode = msg.mode;
    _lowState.motorState[8].q = msg.q;
    _lowState.motorState[8].dq = msg.dq;
    _lowState.motorState[8].tauEst = msg.tauEst;
}

void IOSIM::RLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[9].mode = msg.mode;
    _lowState.motorState[9].q = msg.q;
    _lowState.motorState[9].dq = msg.dq;
    _lowState.motorState[9].tauEst = msg.tauEst;
}

void IOSIM::RLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[10].mode = msg.mode;
    _lowState.motorState[10].q = msg.q;
    _lowState.motorState[10].dq = msg.dq;
    _lowState.motorState[10].tauEst = msg.tauEst;
}

void IOSIM::RLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[11].mode = msg.mode;
    _lowState.motorState[11].q = msg.q;
    _lowState.motorState[11].dq = msg.dq;
    _lowState.motorState[11].tauEst = msg.tauEst;
}