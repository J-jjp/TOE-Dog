/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "common/unitreeRobot.h"
#include <iostream>

Vec3 QuadrupedRobot::getX(LowlevelState &state){
    return getFootPosition(state, 0, FrameType::BODY);
}

Vec34 QuadrupedRobot::getVecXP(LowlevelState &state){
    Vec3 x = getX(state);
    Vec34 vecXP, qLegs;
    qLegs = state.getQ();

    for(int i(0); i < 4; ++i){
        vecXP.col(i) = _Legs[i]->calcPEe2B(qLegs.col(i)) - x;
    }
    return vecXP;
}
// Inverse Kinematics
Vec12 QuadrupedRobot::getQ(const Vec34 &vecP, FrameType frame){
    Vec12 q;
    for(int i(0); i < 4; ++i){
        q.segment(3*i, 3) = _Legs[i]->calcQ(vecP.col(i), frame);
    }
    return q;
}

Vec12 QuadrupedRobot::getQd(const Vec34 &pos, const Vec34 &vel, FrameType frame){
    Vec12 qd;
    for(int i(0); i < 4; ++i){
        qd.segment(3*i, 3) = _Legs[i]->calcQd(pos.col(i), vel.col(i), frame);
    }
    return qd;
}

Vec12 QuadrupedRobot::getTau(const Vec12 &q, const Vec34 feetForce){
    Vec12 tau;
    for(int i(0); i < 4; ++i){
        tau.segment(3*i, 3) = _Legs[i]->calcTau(q.segment(3*i, 3), feetForce.col(i));
    }
    return tau;
}

// Forward Kinematics
Vec3 QuadrupedRobot::getFootPosition(LowlevelState &state, int id, FrameType frame){
    Vec34 qLegs= state.getQ();

    if(frame == FrameType::BODY){
        return _Legs[id]->calcPEe2B(qLegs.col(id));
    }else if(frame == FrameType::HIP){
        return _Legs[id]->calcPEe2H(qLegs.col(id));
    }else{
        std::cout << "[ERROR] The frame of function: getFootPosition can only be BODY or HIP." << std::endl;
        exit(-1);
    }
}

// Forward derivative Kinematics
Vec3 QuadrupedRobot::getFootVelocity(LowlevelState &state, int id){
    Vec34 qLegs = state.getQ();
    Vec34 qdLegs= state.getQd();
    return _Legs[id]->calcVEe(qLegs.col(id), qdLegs.col(id));
}

// Forward Kinematics
Vec34 QuadrupedRobot::getFeet2BPositions(LowlevelState &state, FrameType frame){
    Vec34 feetPos;
    if(frame == FrameType::GLOBAL){
        for(int i(0); i<4; ++i){
            feetPos.col(i) = getFootPosition(state, i, FrameType::BODY);
        }
        feetPos = state.getRotMat() * feetPos;
    }
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
        for(int i(0); i<4; ++i){
            feetPos.col(i) = getFootPosition(state, i, frame);
        }
    }
    else{
        std::cout << "[ERROR] Frame error of function getFeet2BPositions" << std::endl;
        exit(-1);
    }
    return feetPos;
}

Vec34 QuadrupedRobot::getFeet2BVelocities(LowlevelState &state, FrameType frame){
    Vec34 feetVel;
    for(int i(0); i<4; ++i){
        feetVel.col(i) = getFootVelocity(state, i);
    }

    if(frame == FrameType::GLOBAL){
        Vec34 feetPos = getFeet2BPositions(state, FrameType::BODY);
        feetVel += skew(state.getGyro()) * feetPos;
        return state.getRotMat() * feetVel;
    }
    else if((frame == FrameType::BODY) || (frame == FrameType::HIP)){
        return feetVel;
    }
    else{
        std::cout << "[ERROR] Frame error of function getFeet2BVelocities" << std::endl;
        exit(-1);
    }   
}

Mat3 QuadrupedRobot::getJaco(LowlevelState &state, int legID){
    return _Legs[legID]->calcJaco(state.getQ().col(legID));
}

//自己机器人的一些初始化参数
Ask3Robot::Ask3Robot(){
    //param1:腿的id  param2:机身中心到该腿基座坐标系{0}原点的向量
    _Legs[0] = new Ask3Leg(0, Vec3( 0.2625, -0.072, 0));
    _Legs[1] = new Ask3Leg(1, Vec3( 0.2625,  0.072, 0));
    _Legs[2] = new Ask3Leg(2, Vec3(-0.2625, -0.072, 0));
    _Legs[3] = new Ask3Leg(3, Vec3(-0.2625,  0.072, 0));

    _feetPosNormalStand <<  0.2625,  0.2625, -0.2625, -0.2625,
                           -0.1602,  0.1602, -0.1602,  0.1602,
                           -0.33231, -0.33231, -0.33231, -0.33231;

    _robVelLimitX << -0.4, 0.4;
    _robVelLimitY << -0.3, 0.3;
    _robVelLimitYaw << -0.5, 0.5;


#ifdef COMPILE_WITH_REAL_ROBOT
    _mass = 22.036;
    _pcb << 0.002, 0.0, 0.01674;
    _Ib = Vec3(0.3154, 1.2063, 1.3443).asDiagonal();
#endif  // COMPILE_WITH_REAL_ROBOT

#ifdef COMPILE_WITH_SIMULATION
    _mass = 22.036;
    _pcb << 0.002, 0.0, 0.01674;
    _Ib = Vec3(0.3154, 1.2063, 1.3443).asDiagonal();
#endif  // COMPILE_WITH_SIMULATION
}