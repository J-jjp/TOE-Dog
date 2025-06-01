#include "dm_controllers/DmController.h"
//#include <unordered_map>
#define ZERO_COMP 1.5707963267948966

namespace damiao
{

bool DmController::init(HybridJointInterface* robot_hw, ros::NodeHandle& nh)
{
  std::cerr<<"Successfully got HybridEffort joint interface"<<std::endl;
 
   std::vector<std::string> joint_names{
    "joint0_motor","joint1_motor","joint2_motor",
    "joint3_motor","joint4_motor","joint5_motor",
    "joint6_motor","joint7_motor","joint8_motor",
    "joint9_motor","joint10_motor","joint11_motor"
  };
  for (const auto& joint_name : joint_names)
  {
    hybridJointHandles_.push_back(robot_hw->getHandle(joint_name));
  }
  
  dm_cmd_sub_ = nh.subscribe("/dm_cmd", 1, &DmController::commandCallback, this);
  dm_state_pub_ = nh.advertise<damiao_msgs::DmState>("/dm_states", 10);
  
  cached_num_joints_ = hybridJointHandles_.size();
  dm_state_msg_.joint_names.resize(cached_num_joints_);
  dm_state_msg_.pos.resize(cached_num_joints_);
  dm_state_msg_.vel.resize(cached_num_joints_);
  dm_state_msg_.tau.resize(cached_num_joints_);
  dm_cmd_msg_.pos.resize(cached_num_joints_);
  dm_cmd_msg_.vel.resize(cached_num_joints_);
  dm_cmd_msg_.kp.resize(cached_num_joints_);
  dm_cmd_msg_.kd.resize(cached_num_joints_);
  dm_cmd_msg_.tau.resize(cached_num_joints_);
    
  return true;
}

void DmController::starting(const ros::Time& time)
{
 // ROS_INFO("DmController started.");
 std::cerr<<"DmController started."<<std::endl;
}

void DmController::commandCallback(const damiao_msgs::DmCommand::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  
  // // 建立名称到索引的映射
  // static const std::unordered_map<std::string, size_t> name_to_idx{
  //   {"joint0_motor", 0}, {"joint1_motor", 1}, {"joint2_motor", 2}};

  // for (size_t i = 0; i < msg->joint_names.size(); ++i) {
  //   const auto& name = msg->joint_names[i];
  //   if (auto it = name_to_idx.find(name); it != name_to_idx.end()) {
  //     const size_t idx = it->second;
  //     if (i < msg->position.size()) cmd_pos_[idx] = msg->position[i];
  //     if (i < msg->velocity.size()) cmd_vel_[idx] = msg->velocity[i];
  //     if (i < msg->kp.size()) cmd_kp_[idx] = msg->kp[i];
  //     if (i < msg->kd.size()) cmd_kd_[idx] = msg->kd[i];
  //     if (i < msg->effort.size()) cmd_effort_[idx] = msg->effort[i];
  //   }
  // }
  dm_cmd_msg_.pos = msg->pos;
  dm_cmd_msg_.vel = msg->vel;
  dm_cmd_msg_.kp  = msg->kp;
  dm_cmd_msg_.kd  = msg->kd;
  dm_cmd_msg_.tau = msg->tau;
  std::cout<<"dm_cmd_msg_.pos:\t"<<dm_cmd_msg_.pos[0]<<std::endl;
}

void DmController::update(const ros::Time& time, const ros::Duration& period)
{
  // 设置关节命令
 // std::cerr<<"dmcontroller update"<<std::endl;
 //std::cerr<<"size: "<<hybridJointHandles_.size()<<std::endl;
//  float q = sin(std::chrono::system_clock::now().time_since_epoch().count() / 1e9);
//   hybridJointHandles_[0].setCommand(0.0, q*-3.0,0.0,0.3,0.0);
 // hybridJointHandles_[1].setCommand(0.0, q*2.0,0.0,0.3,0.0);
 // hybridJointHandles_[2].setCommand(0.0, q*-5.0,0.0,0.3,0.0);

  const size_t n = cached_num_joints_;
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    for (size_t i = 0; i < n; ++i) {
      if (i == 4 || i == 6 || i == 9 || i == 10)
        hybridJointHandles_[i].setCommand(-dm_cmd_msg_.pos[i], -dm_cmd_msg_.vel[i], dm_cmd_msg_.kp[i], dm_cmd_msg_.kd[i], -dm_cmd_msg_.tau[i]);
      else if (i == 2 || i == 8)
        hybridJointHandles_[i].setCommand(-dm_cmd_msg_.pos[i] - ZERO_COMP, -dm_cmd_msg_.vel[i], dm_cmd_msg_.kp[i], dm_cmd_msg_.kd[i], -dm_cmd_msg_.tau[i]);
      else if (i == 5 || i == 11)
        hybridJointHandles_[i].setCommand(dm_cmd_msg_.pos[i] + ZERO_COMP, dm_cmd_msg_.vel[i], dm_cmd_msg_.kp[i], dm_cmd_msg_.kd[i], dm_cmd_msg_.tau[i]);
      else
        hybridJointHandles_[i].setCommand(dm_cmd_msg_.pos[i], dm_cmd_msg_.vel[i], dm_cmd_msg_.kp[i], dm_cmd_msg_.kd[i], dm_cmd_msg_.tau[i]);
    }
  }
  // std::cerr<<"pos:\t";
  // for (size_t i = 0; i < 12; i++)
  // {
  //   std::cerr<<"\t"<<dm_cmd_msg_.pos[i];
  // }
  // std::cerr<<std::endl;
  
  for (size_t i = 0; i < n; ++i) {
    const auto& joint            = hybridJointHandles_[i];
    dm_state_msg_.joint_names[i] = joint.getName();
    if (i == 4 || i == 6 || i == 9 || i == 10){
      dm_state_msg_.pos[i] = -joint.getPosition();
      dm_state_msg_.vel[i] = -joint.getVelocity();
      dm_state_msg_.tau[i] = -joint.getEffort();
    }
    else if (i == 2 || i == 8){
      dm_state_msg_.pos[i] = -joint.getPosition()-ZERO_COMP;
      dm_state_msg_.vel[i] = -joint.getVelocity();
      dm_state_msg_.tau[i] = -joint.getEffort();
    }
    else if (i == 5 || i == 11){
      dm_state_msg_.pos[i] = joint.getPosition()-ZERO_COMP;
      dm_state_msg_.vel[i] = joint.getVelocity();
      dm_state_msg_.tau[i] = joint.getEffort();
    }
    else{
      dm_state_msg_.pos[i] = joint.getPosition();
      dm_state_msg_.vel[i] = joint.getVelocity();
      dm_state_msg_.tau[i] = joint.getEffort();
    }
  }

dm_state_pub_.publish(dm_state_msg_);
}

void DmController::stopping(const ros::Time& time)
{
 // ROS_INFO("DmController stopped.");
  std::cerr<<"DmController stop."<<std::endl;
}


}  // namespace damiao

// 注册插件
PLUGINLIB_EXPORT_CLASS(damiao::DmController, controller_interface::ControllerBase);