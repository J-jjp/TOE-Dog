/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef ROBOT_TYPE_T2
#include "interface/IOROS_dm.h"
// #include "interface/KeyBoard.h"
// #include "interface/WirelessHandle.h"
#include <iostream>
#include <unistd.h>
#include <csignal>
void IOROS_dm::RosShutDown(int sig){
	ROS_INFO("ROS interface shutting down!");
	ros::shutdown();
}


IOROS_dm::IOROS_dm():IOInterface() {
    initRecv();
    // one threads
    subSpinner.start();


    usleep(300000);     //wait for subscribers start
    // initialize publisher
    initSend();   

    // signal(SIGINT, RosShutDown);
    if (1)
    {
        cmdPanel = new WirelessHandle();
    }
    else{
        cmdPanel = new KeyBoard();
    }
}
IOROS_dm::~IOROS_dm(){

    delete cmdPanel;
    std::cout<<"1111"<<std::endl;

}

void IOROS_dm::sendRecv( LowlevelCmd *cmd, LowlevelState *state) {
    // send(cmd);
    recv(state);
    std::cout<<"pose:\t";
    for (size_t i = 9; i < 12; i++)
    {
        std::cout<<"\t"<<_lowState.motorState[i].q;
        
    }
    std::cout<<std::endl;
    std::cout<<"dp:\t";
    for (size_t i = 9; i < 12; i++)
    {
        std::cout<<"\t"<<_lowState.motorState[i].dq;
        
    }
    std::cout<<std::endl;
    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
}
// cmd
// uint8[] legid    uint8[] motorid     uint8[] mode
// float32[] tau    float32[] vel   float32[] pos
// float32[] kp     float32[] kd
void IOROS_dm::send( LowlevelCmd *cmd) {
    damiao_msgs::DmCommand dm_cmd_msg_;

    // std::cout << dm_cmd_msg_.pos.size() << std::endl;
    for (int i = 0; i <12; i++) {
        dm_cmd_msg_.pos.push_back(cmd->motorCmd[i].q);
        dm_cmd_msg_.tau.push_back(cmd->motorCmd[i].tau);
        dm_cmd_msg_.vel.push_back(cmd->motorCmd[i].dq);
        dm_cmd_msg_.kp.push_back(cmd->motorCmd[i].Kp);
        dm_cmd_msg_.kd.push_back(cmd->motorCmd[i].Kd);
        std::cout<<"p"<<cmd->motorCmd[i].q;
    }

    std::cout<<std::endl;
    _servo_pub.publish(dm_cmd_msg_);
    ros::spinOnce();
}
void IOROS_dm::recv(LowlevelState *state){
    for(int i(0); i < 12; ++i){
        state->motorState[i].q = _lowState.motorState[i].q;
        state->motorState[i].dq = _lowState.motorState[i].dq;
        state->motorState[i].tauEst = _lowState.motorState[i].tauEst;
    }
    for(int i(0); i < 3; ++i){
        state->imu.quaternion[i] = _lowState.imu.quaternion[i];
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
        state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];
    }
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];
    // std::cout<<"kd:"<<state->motorState[3].q;
    // std::cout<<std::endl;
    // std::cout<<"kd:"<<state->motorState[3].dq;
    // std::cout<<std::endl;

    // std::cout<<"kd:"<<state->motorState[4].q;
    // std::cout<<std::endl;

    // std::cout<<"kd:"<<state->motorState[4].dq;
    // std::cout<<std::endl;

    // std::cout<<"kd:"<<state->motorState[5].q;
    // std::cout<<std::endl;

    // std::cout<<"kd:"<<state->motorState[5].dq;
    // std::cout<<std::endl;


    // std::cout<<"dq:"
    // for(int i(0); i < 12; ++i){
    //     std::cout<<"  dq"<<i<<" :"<<state->motorState[i].q;
    // }
    std::cout<<std::endl;
}

void IOROS_dm::initRecv(){
    _imu_sub = _nm.subscribe("/imu", 100, &IOROS_dm::imuCallback, this);
    _servo_sub = _nm.subscribe("/dm_states", 100, &IOROS_dm::MotorStateCallback, this);
}
void IOROS_dm::initSend(){
    _servo_pub = _nm.advertise<damiao_msgs::DmCommand>("/dm_cmd",10);
}

void IOROS_dm::imuCallback(const sensor_msgs::ImuConstPtr& msg){
    _lowState.imu.quaternion[0] = msg->orientation.w;
    _lowState.imu.quaternion[1] = msg->orientation.x;
    _lowState.imu.quaternion[2] = msg->orientation.y;
    _lowState.imu.quaternion[3] = msg->orientation.z;

    _lowState.imu.gyroscope[0] = msg->angular_velocity.x;
    _lowState.imu.gyroscope[1] = msg->angular_velocity.y;
    _lowState.imu.gyroscope[2] = msg->angular_velocity.z;
    
    _lowState.imu.accelerometer[0] = msg->linear_acceleration.x;
    _lowState.imu.accelerometer[1] = msg->linear_acceleration.y;
    _lowState.imu.accelerometer[2] = msg->linear_acceleration.z;
}
void IOROS_dm::MotorStateCallback(const damiao_msgs::DmState::ConstPtr &msg){

    for (size_t i = 0; i < 12; i++)
    {
        // _lowState.motorState[i].mode = msg->mode[i];
        _lowState.motorState[i].q = msg->pos[i];
        _lowState.motorState[i].dq = msg->vel[i];
        _lowState.motorState[i].tauEst = msg->tau[i];
    }
}
Eigen::Vector3d IOROS_dm::quat_rotate_inverse(const Eigen::Vector4d& q, const Eigen::Vector3d& v) {
    double q_w = q[3];  // 提取四元数的实部 w
    Eigen::Vector3d q_vec(q[0], q[1], q[2]);  // 提取四元数的虚部 xyz
    Eigen::Vector3d a = v * (2.0 * q_w * q_w - 1.0);

    // 计算b = cross(q_vec, v) * 2.0 * q_w
    Eigen::Vector3d b = q_vec.cross(v) * 2.0 * q_w;

    // 计算c = q_vec * (q_vec.transpose() * v) * 2.0
    Eigen::Vector3d c = q_vec * (q_vec.transpose() * v) * 2.0;
    return a - b + c;
}
// uint8[] legid
// uint8[] motorid
// uint8[] mode
// float32[] tau
// float32[] vel
// float32[] acc
// float32[] pos
// int8[] temp
// int8[] error
#endif
