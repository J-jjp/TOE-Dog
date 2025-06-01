
#ifndef LOWLEVELSTATE_HPP
#define LOWLEVELSTATE_HPP

#include <iostream>
#include "common/mathTypes.h"
#include "common/mathTools.h"
#include "interface/CmdPanel.h"
#include "common/enumClass.h"

struct MotorState
{
    float q;
    float dq;
    float tauEst;

    MotorState(){
        q = 0;
        dq = 0;
        tauEst = 0;
    }
};
struct Auto_speed{
    float x;    // w, x, y, z 四元素
    float yaw;    //二维码
    float speed_yaw;    //赛道

    Auto_speed(){
        float x=0;    // w, x, y, z 四元素
        float yaw=0;    //三个方向的角速度
        float speed_yaw=0;    //三个方向的角速度

    }
};
struct Auto_barrier{
    float x;    // w, x, y, z 四元素
    float yaw;    //三个方向的角速度
    bool change_next;    //三个方向的角速度

    Auto_barrier(){
        float x=0;    // w, x, y, z 四元素
        float yaw=0;    //三个方向的角速度
        bool change_next=false;    //三个方向的角速度


    }
};
struct IMU
{
    float quaternion[4];    // w, x, y, z 四元素
    float gyroscope[3];    //三个方向的角速度
    float accelerometer[3]; //三个方向的加速度
    float line[3];
    IMU(){
        for(int i = 0; i < 3; i++){
            quaternion[i] = 0;
            gyroscope[i] = 0;
            accelerometer[i] = 0;
        }
        quaternion[3] = 0;
    }

    RotMat getRotMat(){
        Quat quat;
        quat << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return quatToRotMat(quat);
    }

    Vec3 getAcc(){
        Vec3 acc;
        acc << accelerometer[0], accelerometer[1], accelerometer[2];
        return acc;
    }

    Vec3 getGyro(){
        Vec3 gyro;
        gyro << gyroscope[0], gyroscope[1], gyroscope[2];
        return gyro;
    }

    Quat getQuat(){
        Quat q;
        q << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return q;
    }
};
//包含机身12个电机返回的状态以及IMU的状态   在控制器的每一个控制周期中，数据收发接口(IOSIM类和IOTMOTOR类)都会发送一次LowlevelCmd类型的数据，
//并且将最新的状态读取到LowLevelState中。所以在每一个有限状态机的状态下，只需根据LowlevelState计算LowlevelCmd，具体的通信操作都可以交给底层代码完成
struct LowlevelState
{
    IMU imu;
    MotorState motorState[12];
    UserCommand userCmd;
    UserValue userValue;
    Auto_speed speed;
    Auto_barrier barrier;

    Vec34 getQ(){
        Vec34 qLegs;
        for(int i(0); i < 4; ++i){
            qLegs.col(i)(0) = motorState[3*i    ].q;
            qLegs.col(i)(1) = motorState[3*i + 1].q;
            qLegs.col(i)(2) = motorState[3*i + 2].q;
        }
        return qLegs;
    }

    Vec34 getQd(){
        Vec34 qdLegs;
        for(int i(0); i < 4; ++i){
            qdLegs.col(i)(0) = motorState[3*i    ].dq;
            qdLegs.col(i)(1) = motorState[3*i + 1].dq;
            qdLegs.col(i)(2) = motorState[3*i + 2].dq;
        }
        return qdLegs;
    }

    RotMat getRotMat(){
        return imu.getRotMat();
    }

    Vec3 getAcc(){
        return imu.getAcc();
    }

    Vec3 getGyro(){
        return imu.getGyro();
    }

    Vec3 getAccGlobal(){
        return getRotMat() * getAcc();
    }

    Vec3 getGyroGlobal(){
        return getRotMat() * getGyro();
    }

    double getYaw(){
        return rotMatToRPY(getRotMat())(2);
    }

    double getDYaw(){
        return getGyroGlobal()(2);
    }

    void setQ(Vec12 q){
        for(int i(0); i<12; ++i){
            motorState[i].q = q(i);
        }
    }
};

#endif  //LOWLEVELSTATE_HPP