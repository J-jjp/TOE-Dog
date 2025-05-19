#ifndef MY_CONTROLLER_PLUGIN_DM_CONTROLLER_H
#define MY_CONTROLLER_PLUGIN_DM_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dm_common/HybridJointInterface.h>
#include <damiao_msgs/DmCommand.h>
#include <damiao_msgs/DmState.h>
#include <mutex>
namespace damiao
{


class DmController : public controller_interface::Controller<HybridJointInterface>
{
public:
  DmController() = default;
  ~DmController() = default;

  bool init(HybridJointInterface* robot_hw, ros::NodeHandle& nh) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

private:
  std::vector<HybridJointHandle> hybridJointHandles_;
  ros::Subscriber dm_cmd_sub_;
  ros::Publisher dm_state_pub_;
  damiao_msgs::DmCommand dm_cmd_msg_;
  damiao_msgs::DmState dm_state_msg_;
  size_t cached_num_joints_ = 0;
  std::mutex cmd_mutex_; // 线程锁

  void commandCallback(const damiao_msgs::DmCommand::ConstPtr& msg); // 回调函数

};

}  // namespace my_controller_plugin

#endif  // MY_CONTROLLER_PLUGIN_DM_CONTROLLER_H