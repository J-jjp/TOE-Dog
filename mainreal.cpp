
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
    }

    void spin() {
        ros::spin();
    }
public:
  float x=0;
  float y=0;
  float z=0;
  float w=0;
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
    ctrlFrame.run();

  }
ros_thread.join();
  return 1;
}
