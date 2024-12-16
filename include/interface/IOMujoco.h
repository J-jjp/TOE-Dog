/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef IOMUJOCO_H
#define IOMUJOCO_H

#include "interface/IOInterface.h"
#include <mujoco/mujoco.h>
#include <string>

class IOMujoco : public IOInterface{
public:
    IOMujoco(mjData *data)::_data(data){
        std::cout<<"generate interfaces"<<std::endl;
    }
    ~IOMujoco(){};
    float pd_control(float target_q,float q,float kp,float target_dq,float dq,float kd);
    void sendCmd(const LowlevelCmd *cmd, LowlevelState *state);
    void Robot_stateCallback( LowlevelState *state);
private:
    mjData* _data = NULL; 


    //Callback functions for Mujoco
    // void imuCallback(const sensor_msgs::Imu & msg);
    // void FRhipCallback(const unitree_legged_msgs::MotorState& msg);
    // void FRthighCallback(const unitree_legged_msgs::MotorState& msg);
    // void FRcalfCallback(const unitree_legged_msgs::MotorState& msg);

    // void FLhipCallback(const unitree_legged_msgs::MotorState& msg);
    // void FLthighCallback(const unitree_legged_msgs::MotorState& msg);
    // void FLcalfCallback(const unitree_legged_msgs::MotorState& msg);

    // void RRhipCallback(const unitree_legged_msgs::MotorState& msg);
    // void RRthighCallback(const unitree_legged_msgs::MotorState& msg);
    // void RRcalfCallback(const unitree_legged_msgs::MotorState& msg);

    // void RLhipCallback(const unitree_legged_msgs::MotorState& msg);
    // void RLthighCallback(const unitree_legged_msgs::MotorState& msg);
    // void RLcalfCallback(const unitree_legged_msgs::MotorState& msg);
};

#endif  // IOSIM_H