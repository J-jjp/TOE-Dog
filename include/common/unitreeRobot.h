/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef UNITREEROBOT_H
#define UNITREEROBOT_H

#include "common/unitreeLeg.h"
#include "message/LowlevelState.h"

class QuadrupedRobot{
public:
    QuadrupedRobot(){};//构造函数
    ~QuadrupedRobot(){}//析构函数

    Vec3 getX(LowlevelState &state);
    Vec34 getVecXP(LowlevelState &state);

    // Inverse Kinematics(Body/Hip Frame)
    //逆向运动学计算 param1:三行四列的矩阵，每一列分别代表四个足端的位置坐标 param2:代表参数1所在的参考坐标系 只可以是FrameType::HIP 或者FrameType::BODY
    //获取当前机器人全部12个关节的角度
    Vec12 getQ(const Vec34 &feetPosition, FrameType frame);//Vec34中的34代表三行四列
    //逆向微分运动学计算 param1:每一列分别代表四个足端的位置坐标 param2:每一列分别代表四个足端的速度 param3:代表参数1、2所在的参考坐标系 只可以是FrameType::HIP 或者FrameType::BODY
    //获取当前机器人全部12个关节的角速度
    Vec12 getQd(const Vec34 &feetPosition, const Vec34 &feetVelocity, FrameType frame);
    //机器人静力学计算  param1:12个电机的关节角度 param2:四个足端的对外作用力
    //获取当前机器人全部12个关节的力矩
    Vec12 getTau(const Vec12 &q, const Vec34 feetForce);

    // Forward Kinematics
    //获取机器人第id条腿在frame坐标系下的位置坐标
    //LowlevelState类型的结构体包含机器人所有关节角度信息 frame只可以是FrameType::HIP 或者FrameType::BODY
    Vec3 getFootPosition(LowlevelState &state, int id, FrameType frame);
    //获取机器人第id条腿的速度向量    为什么没有参考坐标系？
    Vec3 getFootVelocity(LowlevelState &state, int id);
    //获取所有足端相对于机身中心的位置坐标  frame除了可以是FrameType::HIP 或者FrameType::BODY外，还可以是代表世界坐标系的FrameType::GLOBAL
    Vec34 getFeet2BPositions(LowlevelState &state, FrameType frame);
    //获取所有足端相对于机身中心的速度向量  frame除了可以是FrameType::HIP 或者FrameType::BODY外，还可以是代表世界坐标系的FrameType::GLOBAL
    Vec34 getFeet2BVelocities(LowlevelState &state, FrameType frame);
    //获取第legID条腿在state状态下的雅可比矩阵 不同机器人的参数各不相同  所以会将自己的机器人作为一个子类继承该类
    //在子类的构造函数中会对机器人的一些变量进行初始化
    Mat3 getJaco(LowlevelState &state, int legID);

    Vec2 getRobVelLimitX(){return _robVelLimitX;}
    Vec2 getRobVelLimitY(){return _robVelLimitY;}
    Vec2 getRobVelLimitYaw(){return _robVelLimitYaw;}
    Vec34 getFeetPosIdeal(){return _feetPosNormalStand;}
    double getRobMass(){return _mass;}
    Vec3 getPcb(){return _pcb;}
    Mat3 getRobInertial(){return _Ib;}

protected:
    //类组合 腿类的顶端  _Legs[0]~_Legs[3]分别代表四足的四条腿
    QuadrupedLeg* _Legs[4];
    Vec2 _robVelLimitX; //机器人在机身坐标系{b}下x轴方向的平移速度区间 注意这是一个2维的量,表示一个区间
    Vec2 _robVelLimitY; //机器人在机身坐标系{b}下y轴方向的平移速度区间
    Vec2 _robVelLimitYaw; //机器人在机身坐标系{b}下绕z轴方向的转动角速度区间
    Vec34 _feetPosNormalStand;//代表各个足端中性落脚点在机身坐标系{b}下的坐标
    double _mass; //机器人简化模型的质量
    Vec3 _pcb;
    Mat3 _Ib;
};

class Ask3Robot : public QuadrupedRobot{
public:
    Ask3Robot();
    ~Ask3Robot(){};
};

#endif  // UNITREEROBOT_H