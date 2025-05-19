#include <ros/ros.h>
#include <damiao_msgs/DmCommand.h>
#include <damiao_msgs/DmState.h>
#include <chrono>
#include <thread>

float _percent = 0;    //%
float _duration = 2000;   //steps
float _startPos[12];
float _targetPos[12] = {-0.1,0.8,-1.5, 0.1,0.8,-1.5, -0.1,1.0,-1.5, 0.1,1.0,-1.5};//-0.1,0.8,-1.5 ,0.1,0.8,-1.5,-0.1,1,-1.5, 0.1,1.,-1.5
damiao_msgs::DmState dm_state_msg_;

void commandCallback(const damiao_msgs::DmState::ConstPtr& msg)
{
    dm_state_msg_.pos = msg->pos;
    dm_state_msg_.vel = msg->vel;
    dm_state_msg_.tau = msg->tau;
    //ROS_INFO("Position[1]: %f", msg->pos[1]);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dm_demo");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<damiao_msgs::DmCommand>("/dm_cmd", 10);
    ros::Subscriber dm_state_sub_ = nh.subscribe("/dm_states", 10, commandCallback);
    
    dm_state_msg_.pos.resize(12);
    
    // 配置消息
    damiao_msgs::DmCommand dm_cmd_msg_;
    dm_cmd_msg_.pos = std::vector<double>(12, 0.0);
    dm_cmd_msg_.vel = std::vector<double>(12, 0.0);
    dm_cmd_msg_.kp  = std::vector<double>(12, 40.0);
    dm_cmd_msg_.kd  = std::vector<double>(12, 1.0);
    dm_cmd_msg_.tau = std::vector<double>(12, 0.0);

    ros::Rate rate(200);  // 100Hz发布频率

    while (ros::ok()) {
        ros::spinOnce();
        
        /////位控站立/////
        for (size_t i = 0; i < 12; ++i) {
            _startPos[i] = static_cast<float>(dm_state_msg_.pos[i]);
        }
        ROS_INFO("Position[2]: %f", dm_state_msg_.pos[2]);
        _percent += (float)1/_duration;
        _percent = _percent > 1 ? 1 : _percent;
        for(int j=0; j<12; j++){
            dm_cmd_msg_.pos[j] = (1 - _percent)*_startPos[j] + _percent*_targetPos[j]; 
        }
        /////位控站立/////

        //msg.position[2] = -1.0;

        pub.publish(dm_cmd_msg_);
        rate.sleep();
    }
    return 0;
}