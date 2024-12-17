// /**********************************************************************
//  Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
// ***********************************************************************/
// #define COMPILE_WITH_REAL_ROBOT 1

// #ifdef COMPILE_WITH_REAL_ROBOT

// #include "interface/IOREAL.h"
// #include "interface/WirelessHandle.h"
// #include <stdio.h>
// #include <chrono>

// IOREAL::IOREAL()
//             :_lrmc(), 
//             base_ang_vel_in_base(0.0, 0.0, 0.0), 
//             base_acc_in_base(0.0, 0.0, 0.0),
//             quaternion_in_earth(1.0, 0.0, 0.0, 0.0),
//             mean_init_q(0.0, 0.0, 0.0, 0.0) {
//     std::cout << "The control interface for real robot" << std::endl;
    
//     cmdPanel = new WirelessHandle(_nh);
// 	//IMU相关消息订阅
//     _baseOrientationInEarthSubscriber = _nh.subscribe("/filter/quaternion", 1, &IOREAL::baseOrientationInEarthCallback, this);
//     _baseAngularVelocityInBaseSubscriber = _nh.subscribe("/imu/angular_velocity", 1, &IOREAL::baseAngularVelocityInBaseCallback, this);
//     _baseAccSubscriber = _nh.subscribe("/imu/acceleration", 1, &IOREAL::baseAccInBaseCallback, this);

//     // subSpinner = new ros::AsyncSpinner(1); // one threads
//     // subSpinner->start();
//     usleep(300000);     //wait for subscribers start
// }

// void IOREAL::motorShutDown(){
// 	ROS_INFO("Motor interface shutting down!");
//     _lrmc.disable_all();
// 	ros::shutdown();
// }


// void IOREAL::send(const LowlevelCmd *cmd)
// {
//     remapCmd(cmd); //映射命令消息 将cmd的数据映射到_tmotorCmd上去 然后发送出去
//     _lrmc.send(_motorCmd);

// }

// void IOREAL::recv(const LowlevelCmd *cmd,LowlevelState *state)
// {
//     _lrmc.recv(_motorCmd,_motorState);
//      remapState(state); //映状态信息  将上面_motorState收到的数据映射到state上去

//     ros::spinOnce(); //ros的函数  非阻塞式的处理回调函数

//     // quaternion earth->base   坐标系变换？？
//     wRb = wRe * quaternion_in_earth.toRotationMatrix();
//     Eigen::Quaterniond quaternion_in_world(wRb);
//     state->imu.quaternion[0] = quaternion_in_world.w();
//     state->imu.quaternion[1] = quaternion_in_world.x();
//     state->imu.quaternion[2] = quaternion_in_world.y();
//     state->imu.quaternion[3] = quaternion_in_world.z();
    
//     for(int i=0; i<3; i++){
//         state->imu.gyroscope[i]  = base_ang_vel_in_base(i);
//         state->imu.accelerometer[i] = base_acc_in_base(i);
//     }

//     state->userCmd = cmdPanel->getUserCmd();
//     state->userValue = cmdPanel->getUserValue();



// void IOREAL::sendRecv(const LowlevelCmd *cmd, LowlevelState *state){
//     remapCmd(cmd); //映射命令消息 将cmd的数据映射到_tmotorCmd上去 然后发送出去
//     _start_send_recv_time = getSystemTime();//获得当前系统的时间
    
//     _lrmc.SendRecv(_motorCmd, _motorState); //这里调用了收发函数 这里很重要
    
//     _end_send_recv_time = getSystemTime(); //获取当前系统时间
//     std::cout << "sendRecv Time: " << _end_send_recv_time - _start_send_recv_time << std::endl; //计算命令收发所花的时间
    
//     // if(_start_test1_time - _start_send_recv_time > 5500) {
//     	// std::cout << "sendRecv Time: " << _end_send_recv_time - _start_send_recv_time << std::endl;
//     // }

//     remapState(state); //映状态信息  将上面_tmotorState收到的数据映射到state上去

// 	//发送的指令是保存到LowlevelCmd *cmd里面的  接收状态是保存到LowlevelState *state里面的

//     ros::spinOnce(); //ros的函数  非阻塞式的处理回调函数

//     // quaternion earth->base   坐标系变换？？
//     wRb = wRe * quaternion_in_earth.toRotationMatrix();
//     Eigen::Quaterniond quaternion_in_world(wRb);
//     state->imu.quaternion[0] = quaternion_in_world.w();
//     state->imu.quaternion[1] = quaternion_in_world.x();
//     state->imu.quaternion[2] = quaternion_in_world.y();
//     state->imu.quaternion[3] = quaternion_in_world.z();
    
//     for(int i=0; i<3; i++){
//         state->imu.gyroscope[i]  = base_ang_vel_in_base(i);
//         state->imu.accelerometer[i] = base_acc_in_base(i);
//     }

//     state->userCmd = cmdPanel->getUserCmd();
//     state->userValue = cmdPanel->getUserValue();
// }

// void IOREAL::remapCmd(const LowlevelCmd *cmd) {
//     for(int i=0; i<12; i++) {
//         int cmd_i = _remap_cmd_order[i];
//         _motorCmd[i].mode = cmd->motorCmd[cmd_i].mode;
//         _motorCmd[i].q = cmd->motorCmd[cmd_i].q;
//         _motorCmd[i].dq = cmd->motorCmd[cmd_i].dq;
//         _motorCmd[i].tau = cmd->motorCmd[cmd_i].tau;
//         _motorCmd[i].Kp = cmd->motorCmd[cmd_i].Kp;
//         _motorCmd[i].Kd = cmd->motorCmd[cmd_i].Kd;
//     }
// }

// void IOREAL::remapState(LowlevelState *state) {
//     for(int i=0; i<12; i++) {
//         int state_i = _remap_state_order[i];
//         state->motorState[i].mode = _motorState[state_i].mode;
//         state->motorState[i].q = _motorState[state_i].q;
//         state->motorState[i].dq = _motorState[state_i].dq;
//         state->motorState[i].tauEst = _motorState[state_i].tauEst;
//     }
// }

// void IOREAL::baseOrientationInEarthCallback(const geometry_msgs::QuaternionStamped::ConstPtr &msg) {
//     quaternion_in_earth.x() = msg->quaternion.x;
// 	quaternion_in_earth.y() = msg->quaternion.y;
// 	quaternion_in_earth.z() = msg->quaternion.z;
// 	quaternion_in_earth.w() = msg->quaternion.w;

//     if(!rotation_ew) {
// 		if(rotation_ew_i == 10) {
// 			// 计算wRe
// 			mean_init_q.x() /= 10.0;
// 			mean_init_q.y() /= 10.0;
// 			mean_init_q.z() /= 10.0;
// 			mean_init_q.w() /= 10.0;

// 			wRe = mean_init_q.toRotationMatrix().transpose();

// 			rotation_ew = true;
// 			std::cout << "wRe Calculation Finished." << std::endl;
// 			std::cout << "mean_init_q" <<mean_init_q.x()<<" "<<mean_init_q.y()<<" "<<mean_init_q.z()<<" "<<mean_init_q.w()<<std::endl;
// 			std::cout << "wRe" << wRe << std::endl;
// 		} else {
// 			mean_init_q.x() += quaternion_in_earth.x();
// 			mean_init_q.y() += quaternion_in_earth.y();
// 			mean_init_q.z() += quaternion_in_earth.z();
// 			mean_init_q.w() += quaternion_in_earth.w();

// 			rotation_ew_i++;
// 		}
// 	}
// }

// void IOREAL::baseAngularVelocityInBaseCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
//     base_ang_vel_in_base(0) = msg->vector.x;
// 	base_ang_vel_in_base(1) = msg->vector.y;
// 	base_ang_vel_in_base(2) = msg->vector.z;
// }

// void IOREAL::baseAccInBaseCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
//     base_acc_in_base(0) = msg->vector.x;
//     base_acc_in_base(1) = msg->vector.y;
//     base_acc_in_base(2) = msg->vector.z;
// }

// #endif  // COMPILE_WITH_REAL_ROBOT
