/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef UNITREELEG_H
#define UNITREELEG_H

#include "common/mathTypes.h"
#include "common/enumClass.h"
//将机器人的腿抽象为一个类，并且把机器人运动学与静力学相关的计算抽象为这个类的成员函数
class QuadrupedLeg{
public:
    //legID为腿的编号,l_abad,l_hip,l_knee,机身中心到该腿基座坐标系{0}原点的向量
    QuadrupedLeg(int legID, float abadLinkLength, float hipLinkLength, 
                 float kneeLinkLength, Vec3 pHip2B);
    ~QuadrupedLeg(){}
    Vec3 calcPEe2H(Vec3 q); //公式5.11当关节角度为q时，计算足端到基座坐标系{0}原点的向量
    Vec3 calcPEe2B(Vec3 q);//当关节角度为q时，计算足端到机身中心的向量坐标
    Vec3 calcVEe(Vec3 q, Vec3 qd);//公式5.42当关节角度为q，关节角速度为qd时，计算足端的速度向量
    Vec3 calcQ(Vec3 pEe, FrameType frame);//公式5.42 计算当足端坐标为pEe时腿上三个关节的角度，而frame表示坐标pEe所在的坐标系
    Vec3 calcQd(Vec3 q, Vec3 vEe); //公式5.43 根据当前腿上三个关节的角度和足端速度计算三个关节的角速度
    Vec3 calcQd(Vec3 pEe, Vec3 vEe, FrameType frame);//根据足端坐标pEe，足端速度，以及参考坐标系 计算三个关节的角速度
    Vec3 calcTau(Vec3 q, Vec3 force); //公式5.46 当三个关节角度为q、足端对外作用力为force时，计算该腿三个关节的力矩
    Mat3 calcJaco(Vec3 q);//公式5.42 当三个关节角度为q时，计算该腿的雅可比矩阵
    Vec3 getHip2B(){return _pHip2B;} //这个是？猜测是hip坐标系到机身中心的向量坐标
protected:
    float q1_ik(float py, float pz, float b2y);
    float q3_ik(float b3z, float b4z, float b);
    float q2_ik(float q1, float q3, float px, 
                float py, float pz, float b3z, float b4z);
    float _sideSign;
    const float _abadLinkLength, _hipLinkLength, _kneeLinkLength;
    const Vec3 _pHip2B;
};
//自己机器人腿
class Ask3Leg : public QuadrupedLeg{
public:
    Ask3Leg(const int legID, const Vec3 pHip2B):
        QuadrupedLeg(legID, 0.0882, 0.25, 0.24086, pHip2B){} //三个物理参数 要通过solidworks参数进行测量
    ~Ask3Leg(){}
};

#endif  // UNITREELEG_H