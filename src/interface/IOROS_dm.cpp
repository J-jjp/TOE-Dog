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


IOROS_dm::IOROS_dm():IOInterface(), tfListener(tfBuffer) {
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
    // std::cout<<"pose:\t";
    // for (size_t i = 9; i < 12; i++)
    // {
    //     std::cout<<"\t"<<_lowState.motorState[i].q;
        
    // }
    // std::cout<<std::endl;
    // std::cout<<"dp:\t";
    // for (size_t i = 9; i < 12; i++)
    // {
    //     std::cout<<"\t"<<_lowState.motorState[i].dq;
        
    // }
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
    state->speed.speed_yaw = _lowState.speed.speed_yaw;
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
#ifdef CONTEST_TYPE_SPEED

#endif

#ifdef CONTEST_TYPE_BARRIER

    if (state->barrier.change_next)
    {
        Label_num++;
        state->barrier.change_next=false;
    }

    
#endif
    std::string Label_apriltag =Label+std::to_string(Label_num);

    // std::cout<<"dq:"
    // for(int i(0); i < 12; ++i){
    //     std::cout<<"  dq"<<i<<" :"<<state->motorState[i].q;
    // }
    std::cout<<std::endl;
    try {
            // 获取Tag36h11_2相对于usb_cam的变换
            // geometry_msgs::TransformStamped transform_tag2 = tfBuffer.lookupTransform(
            //     "usb_cam", "Tag36h11_2", ros::Time(0));
            // printTransform("Tag36h11_2", transform_tag2);
            
            // 获取Tag36h11_6相对于usb_cam的变换
            ros::Time now = ros::Time::now();   

            geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform(
                "usb_cam", 
                Label_apriltag, 
                ros::Time(0),  // 要求严格匹配当前时间戳
                ros::Duration(0.005)); // 等待10ms
            ros::Duration time_diff = now - transform.header.stamp;
            if (time_diff > ros::Duration(0.1)) {  // 超过100ms容差
                ROS_WARN_THROTTLE(1.0, "数据过期(%.1fms > 100ms)", time_diff.toSec()*1000);
                walk_x = 0;
                walk_yaw =0;
            }
            else{
                printTransform(Label_apriltag, transform);
            }
        } catch (tf2::TransformException &ex) {
            // ROS_WARN("TF error: %s", ex.what());
            walk_x = 0;
            walk_yaw = 0;
            ros::Duration(0.005).sleep();
            std::cout<<"error"<<walk_x<<std::endl;

        }
#ifdef CONTEST_TYPE_SPEED
        state->speed.x = walk_x;
        state->speed.yaw = walk_yaw;
        std::cout<<"state->speed.x"<<state->speed.x<<"\tyaw:"<<state->speed.yaw<<std::endl;
#endif
#ifdef CONTEST_TYPE_BARRIER
        state->barrier.x = walk_x;
        state->barrier.yaw = walk_yaw;

        std::cout<<"state->barrier.x"<<state->barrier.x<<"\tyaw:"<<state->barrier.yaw<<std::endl;
#endif

}

void IOROS_dm::initRecv(){
    _imu_sub = _nm.subscribe("/imu", 10, &IOROS_dm::imuCallback, this);
    _servo_sub = _nm.subscribe("/dm_states", 10, &IOROS_dm::MotorStateCallback, this);
#ifdef CONTEST_TYPE_SPEED
    speed_sub = _nm.subscribe("/speed", 10, &IOROS_dm::Speed_error, this);
    // last_received_ = ros::Time::now();
    //  check_timer_ = _nm.createTimer(ros::Duration(0.1), &IOROS_dm::checkConnection, this);
#endif


    ros::Duration(1.0).sleep();

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
void IOROS_dm::Speed_error(const std_msgs::Int32::ConstPtr& msg){

    _lowState.speed.speed_yaw=msg->data;
    std::cout<<"speed_yaw:"<<_lowState.speed.speed_yaw<<std::endl;

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
void IOROS_dm::printTransform(const std::string& target, 
                   const geometry_msgs::TransformStamped& transform) {
    // 提取平移信息
    double x = transform.transform.translation.x;
    double y = transform.transform.translation.y;
    double z = transform.transform.translation.z;
    
    // 提取旋转信息并转换为欧拉角
    double roll, pitch, yaw;
    quaternionToEuler(transform.transform.rotation, roll, pitch, yaw);
    
    // ROS_INFO("==== Transform from usb_cam to %s ====", target.c_str());
    // ROS_INFO("move:");
    // ROS_INFO("  X: %.4f (%.1f cm)", x, x*100);
    // ROS_INFO("  Y: %.4f (%.1f cm)", y, y*100);
    // ROS_INFO("  Z: %.4f (%.1f cm)", z, z*100);
    walk_x = z*100;
    walk_yaw = rad2deg(pitch);
    // ROS_INFO("Euler angles:");
    // ROS_INFO("  Roll(X): %.2f deg", rad2deg(roll));
    // ROS_INFO("  Pitch(Y): %.2f deg", rad2deg(pitch));
    // ROS_INFO("  Yaw(Z): %.2f deg", rad2deg(yaw));
    // ROS_INFO("Quaternions:");
    // ROS_INFO("  x: %.4f", transform.transform.rotation.x);
    // ROS_INFO("  y: %.4f", transform.transform.rotation.y);
    // ROS_INFO("  z: %.4f", transform.transform.rotation.z);
    // ROS_INFO("  w: %.4f", transform.transform.rotation.w);
}
void IOROS_dm::quaternionToEuler(const geometry_msgs::Quaternion& q, 
                      double& roll, double& pitch, double& yaw) {
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf_q);
    m.getRPY(roll, pitch, yaw);
}
double IOROS_dm::rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}
void IOROS_dm::checkConnection(const ros::TimerEvent&) {
    // if (is_receiving_ && (ros::Time::now() - last_received_).toSec() > 0.5) {
    //     is_receiving_ = false;
    //     ROS_WARN("Stopped receiving torque data");
    // }
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
