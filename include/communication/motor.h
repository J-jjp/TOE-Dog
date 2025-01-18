#ifndef MOTOR_H
#define MOTOR_H
#include "motor_msg.h"
#include "uart.hpp"
#include <string> // 包含std::string的头文件
#include <memory>
#include <iostream>
#include <algorithm>
#define PI 3.141592653589793f
class motor
{

public:
    motor(int n,float amount_,int dir_,std::shared_ptr<Uart> serial = nullptr) 
    :_n(n),
     dir(dir_),
     amount(amount_),
     serial_(serial)
    {
        if (serial_ == nullptr)
        {
            serial_ = std::make_shared<Uart>("/dev/ttyUSB0");
        }
        cmd.id = n;
        motor_init();
    };
    float recv_pose(float p){
        if (dir == 1)
        {
            return (-zero_pose+p)/9.1;
        }
        else if(dir == -1){
            return (zero_pose-p)/9.1;
        }
        return 0;
    }
    ~motor(){}
    void set_motor(float kp, float kd, float q, float dq, float tau) // 力位混合控制
    {
        cmd.T=std::max(-0.5f, std::min(tau, 0.5f));
        cmd.W = dq;
        cmd.T=cmd.T;
        cmd.K_P=std::max(-0.03f, std::min(kp, 0.03f));
        cmd.K_W=std::max(-8.5f, std::min(kd, 8.5f));
        if (q==0)
        {
            cmd.Pos = 0;
        }
        else{
            cmd.Pos = send_pose(q);
            // std::cout<<"cmd.q"<<cmd.Pos<<std::endl;
        }
        // std::cout<<"cmd.q"<<q<<std::endl;
        motor_sendRecv();
    }
    void set_motor_pose(float pose) // 位置模式
    {   
        cmd.K_P = rotor_kp;
        cmd.K_W = rotor_kd;
        cmd.Pos = pose;
        cmd.W = 0.0;
        cmd.T = 0;
        motor_sendRecv();
    }
    void set_motor_disable(float kd) // 阻尼模式
    {
        cmd.K_P = 0.;
        cmd.K_W = kd;
        cmd.Pos = 0;
        cmd.W = 0.0;
        cmd.T = 0.0;
        motor_sendRecv();
        // usleep(200);
    }
    void set_motor_torque(float t) // 力矩模式
    {
        cmd.K_P = 0.;
        cmd.K_W = 0;
        cmd.Pos = 0;
        cmd.W = 0.0;
        cmd.T = t;
        cmd.T=std::max(-0.5f, std::min(cmd.T, 0.5f));
        motor_sendRecv();
    }
    void set_limt(){
        if (dir == 1)
        {
            zero_pose=start_pose+amount;
            if (_n == 0)
            {
                max_pose = std::max(recv_pose(zero_pose-1*9.1),recv_pose(start_pose+1*9.1));
                min_pose = std::min(recv_pose(zero_pose-1*9.1),recv_pose(start_pose+1*9.1));
            }
            else if (_n ==1)
            {
                max_pose = std::max(recv_pose(zero_pose - 9.1*PI/5),recv_pose(start_pose+PI/2*9.1));
                min_pose = std::min(recv_pose(zero_pose - 9.1*PI/5),recv_pose(start_pose+PI/2*9.1));
            }
            else if(_n==2){
                max_pose = std::max(recv_pose(zero_pose - 3),recv_pose(start_pose-5));
                min_pose = std::min(recv_pose(zero_pose - 3),recv_pose(start_pose-5));
            }      
        }
        else if(dir == -1){
            zero_pose=start_pose-amount;
            if (_n == 0)
            {
                max_pose = std::max(recv_pose(zero_pose+1*9.1),recv_pose(start_pose-1*9.1));
                min_pose = std::min(recv_pose(zero_pose+1*9.1),recv_pose(start_pose-1*9.1));
            }
            if (_n ==1)
            {
                max_pose = std::max(recv_pose(zero_pose + 9.1*PI/5),recv_pose(start_pose-PI/2*9.1));
                min_pose = std::min(recv_pose(zero_pose + 9.1*PI/5),recv_pose(start_pose-PI/2*9.1));
            }
            if(_n==2){
                max_pose = std::max(recv_pose(zero_pose - 3),recv_pose(start_pose+5));
                min_pose = std::min(recv_pose(zero_pose - 3),recv_pose(start_pose+5));
            }  
        }  
    }
public:
    MotorCmd cmd;
    MotorData data;
    std::shared_ptr<Uart> serial_;
    float amount;
    float start_pose= 0;
    int _n;
    int dir;
    float zero_pose;
    float output_kp = 25;
    float output_kd = 0.6;
    float gear_ratio = 9.1;
    float rotor_kp = (output_kp / (gear_ratio * gear_ratio)) / 26.07;
    float rotor_kd = (output_kd / (gear_ratio * gear_ratio)) * 100.0;
    float max_pose;
    float min_pose;
private:

    void motor_sendRecv() // 发送数据
    {
        if (cmd.mode == 10)
        {
            serial_->SendRecv(cmd);
            data = serial_->GetMotorData();
        }
        else
        {
            std::cout << "Please initialize the motor" << std::endl;
        }
    }
    void motor_init(){
        bool start = true;
        while (start)
        {
            std::cout<<"初始化"<<data.Pos<<std::endl;
            cmd.mode = 10;
            cmd.K_P = 0.;
            cmd.K_W = 0;
            cmd.Pos = 0;
            cmd.W = 0.0;
            cmd.T = 0;
            motor_sendRecv();
            if(data.Pos>0.1&&data.Pos<6.2){
                start_pose=data.Pos;
                start=false;
            }
        }
    }
public:
    float send_pose(float _pose){
        if (dir == 1)
        {
            return  _pose*9.1+zero_pose;
        }
        else if(dir == -1){
            return  -_pose*9.1+zero_pose;
        }
        return 0;
    }
    
};
#endif
//使用示例
//1.无最大值和最小值时
// int main(){
//     std::shared_ptr<SerialPort> serial_=std::make_shared<SerialPort>("/dev/ttyUSB0");
//     std::shared_ptr<motor> motor_=std::make_shared<motor>(0,0,serial_);

//     while (1)
//     {
//         motor_->set_motor_torque(0,0.2);
//         std::cout<<"motor.q"<<motor_->data.q<<std::endl;
//     }
    
// }
//2.力位混合控制 以小腿为例
// #define calf_max 28.5448f
// #define calf_min 4.52717f
// #define calf_joint 20.f
// #define calf_start_joint 4.f
// int main(){
//     std::shared_ptr<SerialPort> serial_=std::make_shared<SerialPort>("/dev/ttyUSB0");
//     std::shared_ptr<motor> motor_=std::make_shared<motor>(calf_max,calf_min,serial_);
// //   //calf_max calf_min为小腿范围
//     float _percent = 0;
//     float _duration = 10; 
//     while (1)
//     {
//         _percent += (float)1/_duration;
//         _percent = _percent > 1 ? 1 : _percent;
//         float pose = (1 - _percent)*calf_start_joint + _percent*calf_joint;
//         motor_->set_motor(2,0.015,3,pose,7,0.3);
//     }
// }




