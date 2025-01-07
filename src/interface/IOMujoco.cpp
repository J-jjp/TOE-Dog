/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "interface/IOMujoco.h"
// #include "interface/KeyBoard.h"
// #include "interface/WirelessHandle.h"
#include <iostream>
#include <unistd.h>
#include <csignal>



IOMujoco::~IOMujoco(){
    delete cmdPanel;
}
std::vector<mjtNum> IOMujoco::get_sensor_data(const std::string &sensor_name)
{
  int sensor_id = mj_name2id(_model, mjOBJ_SENSOR, sensor_name.c_str());
  if (sensor_id == -1)
  {
    std::cout << "no found sensor" << std::endl;
    return std::vector<mjtNum>();
  }
  int data_pos = 0;
  for (int i = 0; i < sensor_id; i++)
  {
    data_pos += _model->sensor_dim[i];
  }
  std::vector<mjtNum> sensor_data(_model->sensor_dim[sensor_id]);
  for (int i = 0; i < sensor_data.size(); i++)
  {
    sensor_data[i] = _data->sensordata[data_pos + i];
  }
  return sensor_data;
}
float IOMujoco::pd_control(float target_q,float q,float kp,float target_dq,float dq,float kd){
    float tau=(target_q - q) * kp + (target_dq - dq) * kd;
    return tau;
}
void IOMujoco::sendRecv( LowlevelCmd *cmd, LowlevelState *state) {
    recv(state);
    for (size_t i = 0; i < 12; i++)
    {
        std::cout<<"\t第"<<i<<"条"<< cmd->motorCmd[i].q;
    }
    std::cout<<std::endl;
    for(int i=0; i < 12; i++){
        cmd->motorCmd[i].tau=pd_control(
        cmd->motorCmd[i].q,state->motorState[i].q,cmd->motorCmd[i].Kp,cmd->motorCmd[i].dq,state->motorState[i].dq,cmd->motorCmd[i].Kd);
    }
    // std::cout<<"q"<<cmd->motorCmd[0].q<<"stateq:"<<state->motorState[0].q<<"kd"<<cmd->motorCmd[1].Kd;
    for (size_t i = 0; i < 12; i++)
    {
        std::cout<<"\t第"<<i<<"条"<< state->motorState[i].q;
    }
    std::cout<<std::endl;
    for (size_t i = 0; i < 12; i++)
    {
        std::cout<<"\t第"<<i<<"条"<< cmd->motorCmd[i].tau;
    }
    send(cmd);
    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
}
void IOMujoco::send( LowlevelCmd *lowCmd) {
    std::cout << "send" << std::endl;
    for(int i=0; i < 12; i++){
        _data->ctrl[i] = lowCmd->motorCmd[i].tau;
    }
}

void IOMujoco::recv(LowlevelState *state){
    for (int i = 0; i < 12; i++)
    {
        state->motorState[i].q = _data->sensordata[i];
    }
    for (int i = 12; i < 24; i++)
    {
        state->motorState[i-12].dq = _data->sensordata[i];
    }
    for (int i = 24; i < 36; i++)
    {
        state->motorState[i-24].tauEst = _data->sensordata[i];
    }
    auto base_quat = get_sensor_data( "orientation");
    auto base_accel = get_sensor_data( "linear-acceleration");
    auto base_gyr = get_sensor_data("angular-velocity");
    for(int i=0; i < 3; ++i){
        state->imu.quaternion[i] = base_quat[i];
        state->imu.accelerometer[i] = base_accel[i];
        state->imu.gyroscope[i] = base_gyr[i];
    }
    state->imu.quaternion[3] = base_quat[3];
}

// void IOSIM::imuCallback(const sensor_msgs::Imu & msg)
// { 
//     _lowState.imu.quaternion[0] = msg.orientation.w;
//     _lowState.imu.quaternion[1] = msg.orientation.x;
//     _lowState.imu.quaternion[2] = msg.orientation.y;
//     _lowState.imu.quaternion[3] = msg.orientation.z;

//     _lowState.imu.gyroscope[0] = msg.angular_velocity.x;
//     _lowState.imu.gyroscope[1] = msg.angular_velocity.y;
//     _lowState.imu.gyroscope[2] = msg.angular_velocity.z;
    
//     _lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
//     _lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
//     _lowState.imu.accelerometer[2] = msg.linear_acceleration.z;
// }