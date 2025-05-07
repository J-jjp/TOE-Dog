#ifndef IOROS_H
#define IOROS_H
#include "ros/ros.h"
#include "control/leg_control.h"
#include "interface/KeyBoard.h"
#include "interface/WirelessHandle.h"
#include "interface/IOInterface.h"
#include "unitree_a1/MotorCmd.h"
#include "unitree_a1/MotorData.h"

#include <sensor_msgs/Imu.h>
#include <string>
class IOROS : public IOInterface{
public:
    IOROS();
    void sendRecv(LowlevelCmd *cmd, LowlevelState *state);
    void send(LowlevelCmd *cmd);
    void recv(LowlevelState *state);
    ros::NodeHandle _nm;
    ros::Subscriber _servo_sub, _imu_sub;
    ros::Publisher _servo_pub;
    LowlevelCmd _lowCmd;
    LowlevelState _lowState; 
    unitree_a1::MotorCmd pub_data;

    ros::AsyncSpinner subSpinner{1}; 
    ~IOROS();
public:
    void initRecv();
    void initSend();
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);
    void MotorStateCallback(const unitree_a1::MotorData::ConstPtr &msg);
    void RosShutDown(int sig);

};

#endif