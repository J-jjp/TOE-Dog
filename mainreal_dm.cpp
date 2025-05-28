
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

#include "interface/IOROS_dm.h"

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
bool running = true;
void ShutDown(int sig)
{
    std::cout << "stop the controller" << std::endl;
    running = false;
    ros::shutdown();

}

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
    setProcessScheduler();
    ros::init(argc, argv, "REAL");
    // ros::start();
    // ros::Rate loop_rate(50);

    LowlevelCmd *lowCmd = new LowlevelCmd();
    LowlevelState *lowState = new LowlevelState();
    // setProcessScheduler();
    IOInterface *ioInter;
    CtrlPlatform ctrlPlat;
    
    ioInter = new IOROS_dm();
    ctrlPlat = CtrlPlatform::REALROBOT;
    CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->dt = 0.02; 
    ctrlComp->running = &running;
    ctrlComp->robotModel = new ToeRobot();
    ControlFrame ctrlFrame(ctrlComp);
    signal(SIGINT, ShutDown);
    while (running) {
        ctrlFrame.run();
    }
    //汪闫岩到此一游    2025.5.24    我是高玩！
    delete ctrlComp;
    return 0;
}
