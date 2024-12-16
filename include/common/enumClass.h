
#ifndef ENUMCLASS_H
#define ENUMCLASS_H

#include <iostream>
#include <sstream>
//控制平台
enum class CtrlPlatform{
    Mujoco,
    REALROBOT
};
//机器人类型
enum class RobotType{
    TOE_Dog,
    GO2
};
//输入指令
enum class UserCommand{
    // EXIT,
    NONE,
    L2_A,       // fixedStand
    L2_B       // passive
};

// enum class FrameType{
//     BODY,
//     HIP,
//     GLOBAL
// };

// enum class WaveStatus{
//     STANCE_ALL,
//     SWING_ALL,
//     WAVE_ALL
// };
//状态机标志位
enum class FSMMode{
    NORMAL,
    CHANGE
};
//状态机类型
enum class FSMStateName{
    // EXIT,
    INVALID,
    PASSIVE,
    FIXEDSTAND,

};

#endif  // ENUMCLASS_H