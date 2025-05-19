#ifndef IOROS_DM_H
#define IOROS_DM_H
#include "ros/ros.h"
#include "control/leg_control.h"
#include "interface/KeyBoard.h"
#include "interface/WirelessHandle.h"
#include "interface/IOInterface.h"
#include <damiao_msgs/DmCommand.h>
#include <damiao_msgs/DmState.h>

#include <sensor_msgs/Imu.h>
#include <string>
class IOROS_dm : public IOInterface{
public:
    IOROS_dm();
    void sendRecv(LowlevelCmd *cmd, LowlevelState *state);
    void send(LowlevelCmd *cmd);
    void recv(LowlevelState *state);
    ros::NodeHandle _nm;
    ros::Subscriber _servo_sub, _imu_sub;
    ros::Publisher _servo_pub;
    LowlevelCmd _lowCmd;
    LowlevelState _lowState; 
    // damiao_msgs::DmCommand dm_cmd_msg_;
    
    Eigen::Vector3d quat_rotate_inverse(const Eigen::Vector4d& q, const Eigen::Vector3d& v);

    ros::AsyncSpinner subSpinner{1}; 
    ~IOROS_dm();
public:
    void initRecv();
    void initSend();
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);
    void MotorStateCallback(const damiao_msgs::DmState::ConstPtr &msg);
    void RosShutDown(int sig);

};

#endif