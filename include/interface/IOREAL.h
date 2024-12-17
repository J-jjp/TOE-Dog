// /**********************************************************************
//  Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
// ***********************************************************************/
// #ifndef IOREAL_H
// #define IOREAL_H

// #include "interface/IOInterface.h"
// //这个添加的是自己的
// //#include "tmotor_controller/MotorController.h"

// #include "LegRobotMotorControl.h"

// #include <ros/ros.h>
// #include <geometry_msgs/QuaternionStamped.h>
// #include <geometry_msgs/Vector3Stamped.h>

// #include "common/timeMarker.h"




// class IOREAL : public IOInterface{
//     public:
//     	IOREAL();
//     	~IOREAL(){}
//     	void motorShutDown();
//     	void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);
// 		//liu tao add---------------------------------
// 		void send(const LowlevelCmd *cmd);
// 		void recv(const LowlevelCmd *cmd,LowlevelState *state);
// 		//liu tao add---------------------------------


//     	void remapCmd(const LowlevelCmd *cmd);
//     	void remapState(LowlevelState *state);

//    		void baseOrientationInEarthCallback(const geometry_msgs::QuaternionStamped::ConstPtr &msg);
//     	void baseAngularVelocityInBaseCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
//     	void baseAccInBaseCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);

//     private:

	
//     	//MotorController _mc; //这个是自己写的控制器
//    		//TmotorCmd _tmotorCmd[12]; //命令消息
//     	//TmotorState _tmotorState[12]; //状态消息

// 		LegRobotMotorControl _lrmc; //修改的底层控制器
// 		MitMotorCmd _motorCmd[12];
// 		MitMotorState _motorState[12];

//     	int _remap_cmd_order[12] = {3, 4, 5, 9, 10, 11, 0, 1, 2, 6, 7, 8}; //命令映射顺序
//     	int _remap_state_order[12] = {6, 7, 8, 0, 1, 2, 9, 10, 11, 3, 4, 5}; //状态映射顺序

//     	ros::NodeHandle _nh;
//     // ros::AsyncSpinner* subSpinner;

//     	ros::Subscriber _baseOrientationInEarthSubscriber;
//     	ros::Subscriber _baseAngularVelocityInBaseSubscriber;
//     	ros::Subscriber _baseAccSubscriber;
//     	Vec3 base_ang_vel_in_base;
//     	Vec3 base_acc_in_base;
//     	Eigen::Quaterniond quaternion_in_earth;
//     	Eigen::Quaterniond mean_init_q;
//     	Eigen::Matrix3d wRe;
//     	Eigen::Matrix3d wRb;
//     	bool rotation_ew = false;
//     	int rotation_ew_i = 0;

//     	long long _start_send_recv_time; //计算开始结束时间的
//     	long long _end_send_recv_time;

// };

// #endif  // IOREAL_H
