
#ifndef LEG_CONTROL_H
#define LEG_CONTROL_H
#include <vector>
#include "motor.h"

#define calf_ammount (PI-0.3628)*9.1
#define thigh_ammount (-PI/2+0.3628)*9.1
class leg_control
{

public:
    leg_control(const std::string& leg_name, const std::string& port,std::vector<int>& arr)
           : _leg_name(leg_name),
             _leg_serial(std::make_shared<Uart>(port))  // 初始化 SerialPort 对象
    {
      _leg_motor0=std::make_shared<motor>(0,0,arr[0],_leg_serial);
      _leg_motor1=std::make_shared<motor>(1,thigh_ammount,arr[1],_leg_serial);
      _leg_motor2=std::make_shared<motor>(2,calf_ammount,arr[2],_leg_serial);
      leg_init();
      leg_start[0]=_leg_motor0->recv_pose(_leg_motor0->start_pose);
      leg_start[1]=_leg_motor1->recv_pose(_leg_motor1->start_pose);
      leg_start[2]=_leg_motor2->recv_pose(_leg_motor2->start_pose);
    };
    ~leg_control(){
    };
    void set_motor_cmd(std::vector<float>& kp, std::vector<float>& kd,
    std::vector<float>& q, std::vector<float>& dq, std::vector<float>& tau);    //控制腿中的一个电机
    void set_leg_disable(float kd);//阻尼模式
    void set_leg_pose(std::vector<float>& arr);  //位置控制
    void set_leg_torque(std::vector<float>& arr); //力矩控制
    void read_data(); //获得整条腿数据
    void recv(); //获得整条腿数据
    bool motor_limit(std::vector<float>& kp,std::shared_ptr<motor> moto);
    void print_pose(); 
    void leg_init();
public:
    std::string _leg_name;            // 定义腿的名称
    std::shared_ptr<Uart> _leg_serial;  // 定义 SerialPort 对象
    std::shared_ptr<motor> _leg_motor0;           // 定义三个个 Motor 对象
    std::shared_ptr<motor> _leg_motor1; 
    std::shared_ptr<motor> _leg_motor2; 
    MotorData leg_data[3];
    float leg_pose[3];
    float leg_W[3];
    float leg_T[3];
    float leg_start[3];
    std::vector<float> leg_speed[3];
};



#endif