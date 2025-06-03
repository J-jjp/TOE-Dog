#ifdef ROBOT_TYPE_T2

#ifndef IOROS_DM_H
#define IOROS_DM_H
#include "ros/ros.h"
#include "control/leg_control.h"
#include "interface/KeyBoard.h"
#include "interface/WirelessHandle.h"
#include "interface/IOInterface.h"
#include <damiao_msgs/DmCommand.h>
#include <damiao_msgs/DmState.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/Int32.h>

#include <cmath>
#include <sensor_msgs/Imu.h>
#include <string>
class IOROS_dm : public IOInterface{
public:
    IOROS_dm();
    void sendRecv(LowlevelCmd *cmd, LowlevelState *state);
    void send(LowlevelCmd *cmd);
    void recv(LowlevelState *state);
    ros::NodeHandle _nm;
    ros::Subscriber _servo_sub, _imu_sub ;
#ifdef CONTEST_TYPE_SPEED
    ros::Subscriber speed_sub;
    ros::Timer check_timer_;
    ros::Time last_received_;
    bool is_receiving_ = false;
#endif
    ros::Publisher _servo_pub;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    LowlevelCmd _lowCmd;
    LowlevelState _lowState; 
    double walk_x = 0;
    double walk_yaw = 0;
    std::string Label ="Tag36h11_";
    int Label_num=10;
    int slope=1;
    int Vertical_bar=10;
    // damiao_msgs::DmCommand dm_cmd_msg_; 
    bool tf_take_over=false;
    Eigen::Vector3d quat_rotate_inverse(const Eigen::Vector4d& q, const Eigen::Vector3d& v);
    void printTransform(const std::string& target, const geometry_msgs::TransformStamped& transform);
    ros::AsyncSpinner subSpinner{1}; 
    ~IOROS_dm();
public:
    void initRecv();
    void initSend();
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);
    void MotorStateCallback(const damiao_msgs::DmState::ConstPtr &msg);
    void quaternionToEuler(const geometry_msgs::Quaternion& q, 
                      double& roll, double& pitch, double& yaw);
    double rad2deg(double rad);
    void RosShutDown(int sig);
    void Speed_error(const std_msgs::Int32::ConstPtr& msg);
    void checkConnection(const ros::TimerEvent&);



};

#endif

#endif
