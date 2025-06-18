/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef LOWLEVELCMD_H
#define LOWLEVELCMD_H

#include "common/mathTypes.h"
#include "common/mathTools.h"

struct MotorCmd_user{
    float q;
    float dq;
    float tau;
    float Kp;
    float Kd;

    MotorCmd_user(){
        q = 0;
        dq = 0;
        tau = 0;
        Kp = 0;
        Kd = 0;
    }
};
//该结构体是控制器发送给机身12个电机的命令
struct LowlevelCmd{
    MotorCmd_user motorCmd[12];

    void setQ(Vec12 q){
        for(int i(0); i<12; ++i){
            motorCmd[i].q = q(i);
        }
    }
    void setQ(int legID, Vec3 qi){
        motorCmd[legID*3+0].q = qi(0);
        motorCmd[legID*3+1].q = qi(1);
        motorCmd[legID*3+2].q = qi(2);
    }
    void setQd(Vec12 qd){
        for(int i(0); i<12; ++i){
            motorCmd[i].dq = qd(i);
        }
    }
    void setQd(int legID, Vec3 qdi){
        motorCmd[legID*3+0].dq = qdi(0);
        motorCmd[legID*3+1].dq = qdi(1);
        motorCmd[legID*3+2].dq = qdi(2);
    }
    void setTau(Vec12 tau, Vec2 torqueLimit = Vec2(-25, 25)){
        for(int i(0); i<12; ++i){
            if(std::isnan(tau(i))){
                printf("[ERROR] The setTau function meets Nan\n");
                motorCmd[i].tau = 0;
            } else { 
                motorCmd[i].tau = saturation(tau(i), torqueLimit);
            }
        }
    }
    void setZeroDq(int legID){
        motorCmd[legID*3+0].dq = 0;
        motorCmd[legID*3+1].dq = 0;
        motorCmd[legID*3+2].dq = 0;
    }
    void setZeroDq(){
        for(int i(0); i<4; ++i){
            setZeroDq(i);
        }
    }
    void setZeroTau(int legID){
        motorCmd[legID*3+0].tau = 0;
        motorCmd[legID*3+1].tau = 0;
        motorCmd[legID*3+2].tau = 0;
    }
    void setSimStanceGain(int legID){
        motorCmd[legID*3+0].Kp = 30;
        motorCmd[legID*3+0].Kd = 0.75;
        motorCmd[legID*3+1].Kp = 50;
        motorCmd[legID*3+1].Kd = 1.25;
        motorCmd[legID*3+2].Kp = 60;
        motorCmd[legID*3+2].Kd = 1.5;
    }
    void setSimjumpeGain(int legID){
        motorCmd[legID*3+0].Kp = 40;
        motorCmd[legID*3+0].Kd = 0.75;
        motorCmd[legID*3+1].Kp = 80;
        motorCmd[legID*3+1].Kd = 2;
        motorCmd[legID*3+2].Kp = 80;
        motorCmd[legID*3+2].Kd = 2;
    }
    void setRealStanceGain(int legID){

        motorCmd[legID*3+0].Kp =30;
        motorCmd[legID*3+0].Kd = 0.75;
        motorCmd[legID*3+1].Kp =50;
        motorCmd[legID*3+1].Kd = 1.25;
        motorCmd[legID*3+2].Kp = 70;
        motorCmd[legID*3+2].Kd = 1.75;
        if (legID>1)
        {
            motorCmd[legID*3+0].Kp =50;
            motorCmd[legID*3+0].Kd = 1.25;
            motorCmd[legID*3+1].Kp =70;
            motorCmd[legID*3+1].Kd = 1.75;
            motorCmd[legID*3+2].Kp = 90;
            motorCmd[legID*3+2].Kd = 2.25;
        }
        
    }
    void setZeroGain(int legID){
        motorCmd[legID*3+0].Kp = 0;
        motorCmd[legID*3+0].Kd = 0;
        motorCmd[legID*3+1].Kp = 0;
        motorCmd[legID*3+1].Kd = 0;
        motorCmd[legID*3+2].Kp = 0;
        motorCmd[legID*3+2].Kd = 0;
    }
    void setZeroGain(){
        for(int i(0); i<4; ++i){
            setZeroGain(i);
        }
    }
    void setStableGain(int legID){
        motorCmd[legID*3+0].Kp = 0.8;
        motorCmd[legID*3+0].Kd = 0.8;
        motorCmd[legID*3+1].Kp = 0.8;
        motorCmd[legID*3+1].Kd = 0.8;
        motorCmd[legID*3+2].Kp = 0.8;
        motorCmd[legID*3+2].Kd = 0.8;
    }
    void setStableGain(){
        for(int i(0); i<4; ++i){
            setStableGain(i);
        }
    }
    void setSwingGain(int legID){
        motorCmd[legID*3+0].Kp = 3;
        motorCmd[legID*3+0].Kd = 2;
        motorCmd[legID*3+1].Kp = 3;
        motorCmd[legID*3+1].Kd = 2;
        motorCmd[legID*3+2].Kp = 3;
        motorCmd[legID*3+2].Kd = 2;
    }
    void setSimrlGain(int legID){
        motorCmd[legID*3+0].Kp = 20;
        motorCmd[legID*3+0].Kd = 0.5;
        motorCmd[legID*3+1].Kp = 20;
        motorCmd[legID*3+1].Kd = 0.5;
        motorCmd[legID*3+2].Kp = 20;
        motorCmd[legID*3+2].Kd = 0.5;
    }
    void setSimbackfileGain(int legID){
        motorCmd[legID*3+0].Kp = 70;
        motorCmd[legID*3+0].Kd = 3;
        motorCmd[legID*3+1].Kp = 70;
        motorCmd[legID*3+1].Kd = 3;
        motorCmd[legID*3+2].Kp = 70;
        motorCmd[legID*3+2].Kd = 3;
    }
    void setRealrlGain(int legID){
        motorCmd[legID*3+0].Kp = 20;
        motorCmd[legID*3+0].Kd = 0.5;
        motorCmd[legID*3+1].Kp = 20;
        motorCmd[legID*3+1].Kd = 0.5;
        motorCmd[legID*3+2].Kp = 20;
        motorCmd[legID*3+2].Kd = 0.5;
    }
    float realrlGain_kp(float kp){
        float kp_new = kp/2048;
        return kp_new;
    }
    float realrlGain_kd(float kd){
        float kd_new = kd*1.5;
        return kd_new;
    }
};

#endif  //LOWLEVELCMD_H
