
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdio>
#include <cstring>
#include "interface/IOReal.h"
#include <control/CtrlComponents.h>
#include <FSM/FSM.h>
#include <string>
#include <iostream>
#include <control/ControlFrame.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <thread>
Eigen::Vector3d quat_rotate_inverse(const Eigen::Vector4d& q, const Eigen::Vector3d& v) {
    double q_w = q[3];  // 提取四元数的实部 w
    Eigen::Vector3d q_vec(q[0], q[1], q[2]);  // 提取四元数的虚部 xyz
    Eigen::Vector3d a = v * (2.0 * q_w * q_w - 1.0);

    // 计算b = cross(q_vec, v) * 2.0 * q_w
    Eigen::Vector3d b = q_vec.cross(v) * 2.0 * q_w;

    // 计算c = q_vec * (q_vec.transpose() * v) * 2.0
    Eigen::Vector3d c = q_vec * (q_vec.transpose() * v) * 2.0;
    return a - b + c;
}

class ImuSubscriber {
public:
    ImuSubscriber() : nh_("~") {
        // 订阅IMU话题
        imu_sub_ = nh_.subscribe("/imu", 10, &ImuSubscriber::imuCallback, this);
    }

    void imuCallback(const sensor_msgs::ImuConstPtr& msg) {
        // 打印接收到的IMU数据
        x=msg->orientation.x;
        y=msg->orientation.y;
        z=msg->orientation.z;
        w=msg->orientation.w;
        gy_x=msg->angular_velocity.x;
        gy_y=msg->angular_velocity.y;
        gy_z=msg->angular_velocity.z;
        acc_x=msg->linear_acceleration.x;
        acc_y=msg->linear_acceleration.y;
        acc_z=msg->linear_acceleration.z;
    }

    void spin() {
        ros::spin();
    }
public:
  float x=0;
  float y=0;
  float z=0;
  float w=0;
  float gy_x=0;
  float gy_y=0;
  float gy_z=0;
  float acc_x=0;
  float acc_y=0;
  float acc_z=0;
private:
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
};
bool running = true;
void setProcessScheduler()
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}
Vec3 quaternion_to_euler_array(Eigen::Quaterniond quat){
    double x = quat.x();
    double y = quat.y();
    double z = quat.z();
    double w = quat.w();
    double t0, t1, t2, t3,t4;
    double roll_x, pitch_y, yaw_z;
    t0 = +2.0 * (w * x + y * z);
    t1 = +1.0 - 2.0 * (x * x + y * y);
    roll_x = std::atan2(t0, t1);
    
    t2 = +2.0 * (w * y - z * x);
    t2 = std::max(-1.0, std::min(t2, 1.0));
    pitch_y =  std::asin(t2);
    
    t3 = +2.0 * (w * z + x * y);
    t4 = +1.0 - 2.0 * (y * y + z * z);
    yaw_z =  std::atan2(t3, t4);
    return {roll_x, pitch_y, yaw_z};
}
// main function
int main(int argc, char** argv) {
    LowlevelCmd *lowCmd = new LowlevelCmd();
    LowlevelState *lowState = new LowlevelState();
    setProcessScheduler();
    IOInterface *ioInter;
    CtrlPlatform ctrlPlat;
    ioInter = new IOReal();
    ctrlPlat = CtrlPlatform::REALROBOT;
    CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->dt = 0.0025; 
    ctrlComp->running = &running;
    ctrlComp->robotModel = new ToeRobot();

    ControlFrame ctrlFrame(ctrlComp);

    ros::init(argc, argv, "imu_subscriber");
    ImuSubscriber imu_subscriber;
    // 创建一个线程来处理ROS消息回调
    std::thread ros_thread(&ImuSubscriber::spin, &imu_subscriber);
  while (ros::ok()) {
    ctrlComp->lowState->imu.quaternion[0]=imu_subscriber.w;
    ctrlComp->lowState->imu.quaternion[1]=imu_subscriber.x;
    ctrlComp->lowState->imu.quaternion[2]=imu_subscriber.y;
    ctrlComp->lowState->imu.quaternion[3]=imu_subscriber.z;
    ctrlComp->lowState->imu.gyroscope[0]=imu_subscriber.gy_x;
    ctrlComp->lowState->imu.gyroscope[1]=imu_subscriber.gy_y;
    ctrlComp->lowState->imu.gyroscope[2]=imu_subscriber.gy_z;
    ctrlComp->lowState->imu.gyroscope[0]=imu_subscriber.acc_x;
    ctrlComp->lowState->imu.gyroscope[1]=imu_subscriber.acc_y;
    ctrlComp->lowState->imu.gyroscope[2]=imu_subscriber.acc_z;
    ctrlFrame.run();
    Eigen::Vector3d v(0.0,0.0,-1.0); 
    Eigen::Vector4d q(imu_subscriber.x,imu_subscriber.y,imu_subscriber.z,imu_subscriber.w);
    Eigen::Vector3d proj_gravity_eigen = quat_rotate_inverse(q, v);
    std::cout<<"222222"<<proj_gravity_eigen[1]<<" "<<-proj_gravity_eigen[0]<<" "<<proj_gravity_eigen[2]<<std::endl;
    usleep(18000);
  }
ros_thread.join();
  return 1;
}