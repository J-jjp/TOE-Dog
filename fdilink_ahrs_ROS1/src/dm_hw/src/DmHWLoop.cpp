
//
// Created by qiayuan on 1/24/22.
//

/********************************************************************************
Modified Copyright (c) 2024-2025, Dm Robotics. 
********************************************************************************/

#include "dm_hw/DmHWLoop.h"

namespace damiao
{
DmHWLoop::DmHWLoop(ros::NodeHandle& nh, std::shared_ptr<DmHW> hardware_interface)
  : nh_(nh), hardwareInterface_(std::move(hardware_interface)), loopRunning_(true)
{
  controllerManager_.reset(new controller_manager::ControllerManager(hardwareInterface_.get(), nh_));

  int error = 0;
  int threadPriority = 0;
  ros::NodeHandle nhP("damiao");
  error += static_cast<int>(!nhP.getParam("loop_frequency", loopHz_));
  error += static_cast<int>(!nhP.getParam("cycle_time_error_threshold", cycleTimeErrorThreshold_));
  error += static_cast<int>(!nhP.getParam("thread_priority", threadPriority));
  if (error > 0)
  {
    std::string error_message =
        "could not retrieve one of the required parameters: damiao/loop_hz or damiao/cycle_time_error_threshold or damiao/thread_priority";
        
    ROS_ERROR_STREAM(error_message);
    throw std::runtime_error(error_message);
  }

  lastTime_ = Clock::now();
  //dm_state_pub_ = nh_.advertise<damiao_msgs::DmState>("dm_states", 10);

  loopThread_ = std::thread([&]() {
    while (loopRunning_)
    {
      update();
    }
  });
  sched_param sched{ .sched_priority = threadPriority };
  if (pthread_setschedparam(loopThread_.native_handle(), SCHED_FIFO, &sched) != 0)
  {
    ROS_WARN(
        "Failed to set threads priority (one possible reason could be that the user and the group permissions "
        "are not set properly.).\n");
  }
}

void DmHWLoop::update()
{
  const auto currentTime = Clock::now();

  const Duration desiredDuration(1.0 / loopHz_);

  Duration time_span = std::chrono::duration_cast<Duration>(currentTime - lastTime_);
  elapsedTime_ = ros::Duration(time_span.count());
  lastTime_ = currentTime;

  const double cycle_time_error = (elapsedTime_ - ros::Duration(desiredDuration.count())).toSec();
  if (cycle_time_error > cycleTimeErrorThreshold_)
  {
    ROS_WARN_STREAM("Cycle time exceeded error threshold by: " << cycle_time_error - cycleTimeErrorThreshold_ << "s, "
                                                               << "cycle time: " << elapsedTime_ << "s, "
                                                               << "threshold: " << cycleTimeErrorThreshold_ << "s");
  }

  hardwareInterface_->read(ros::Time::now(), elapsedTime_);

  controllerManager_->update(ros::Time::now(), elapsedTime_);
  
  hardwareInterface_->write(ros::Time::now(), elapsedTime_);

  // //遍历所有端口和电机发布数据
  // const auto& port_data_map = hardwareInterface_->getPortDataMap();
  // for (const auto& port_pair : port_data_map) {
  //   const std::string& port_name = port_pair.first;
  //   const auto& motor_map = port_pair.second;

  //   damiao_msgs::DmState msg;
  //   msg.port_name = port_name;
  //   // 填充数组数据（包括 joint_names）
  //   for (const auto& motor_pair : motor_map) {
  //     const DmActData& data = motor_pair.second;
  //     msg.motor_id.push_back(motor_pair.first);
  //     msg.joint_names.push_back(data.name);  // 从DmActData中获取关节名
  //     msg.positions.push_back(data.pos);
  //     msg.velocities.push_back(data.vel);
  //     msg.efforts.push_back(data.effort);
  //   }

  //   dm_state_pub_.publish(msg);  // 发布包含关节名的消息
  // }

  const auto sleepTill = currentTime + std::chrono::duration_cast<Clock::duration>(desiredDuration);
  std::this_thread::sleep_until(sleepTill);
}

DmHWLoop::~DmHWLoop()
{
  loopRunning_ = false;
  if (loopThread_.joinable())
  {
    loopThread_.join();
  }
}

}  // namespace damiao
