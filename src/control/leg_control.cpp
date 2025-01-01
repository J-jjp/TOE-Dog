#include "control/leg_control.h"


void leg_control::set_motor_cmd(std::vector<float>& kp, std::vector<float>& kd,
std::vector<float>& q, std::vector<float>& dq, std::vector<float>& tau){
    if(motor_limit(kp,_leg_motor0)){
        _leg_motor0->set_motor(kp[0],kd[0],q[0],dq[0],tau[0]);
    }
    if(motor_limit(kp,_leg_motor1)){
        _leg_motor1->set_motor(kp[1],kd[1],q[1],dq[1],tau[1]);
    }
    if(motor_limit(kp,_leg_motor2)){
        _leg_motor2->set_motor(kp[2],kd[2],q[2],dq[2],tau[2]);
    }
    read_data();
    recv();
}
void leg_control::set_leg_disable(float kd){
    _leg_motor0->set_motor_disable(kd);
    _leg_motor1->set_motor_disable(kd);
    _leg_motor2->set_motor_disable(kd);
    read_data();
    recv();
}
void leg_control::set_leg_pose(std::vector<float>& arr){

}
void leg_control::set_leg_torque(std::vector<float>& arr){

}

void leg_control::read_data(){
    leg_data[(int)_leg_motor0->data.motor_id]=_leg_motor0->data;
    leg_data[(int)_leg_motor1->data.motor_id]=_leg_motor1->data;
    leg_data[(int)_leg_motor2->data.motor_id]=_leg_motor2->data;
}
void leg_control::recv(){
    leg_pose[0]=_leg_motor0->recv_pose(leg_data[0].Pos);
    leg_pose[1]=_leg_motor1->recv_pose(leg_data[1].Pos);
    leg_pose[2]=_leg_motor2->recv_pose(leg_data[2].Pos);
    leg_W[0]=_leg_motor0->recv_pose(leg_data[0].W);
    leg_W[1]=_leg_motor1->recv_pose(leg_data[1].W);
    leg_W[2]=_leg_motor2->recv_pose(leg_data[2].W);
    leg_T[0]=_leg_motor0->recv_pose(leg_data[0].T);
    leg_T[1]=_leg_motor1->recv_pose(leg_data[1].T);
    leg_T[2]=_leg_motor2->recv_pose(leg_data[2].T);
}
void leg_control::print_pose(){
    std::cout<<_leg_name<<"0:"<<leg_pose[0]<<"\t\t1:"<<leg_pose[1]<<"\t2:"<<leg_pose[2]<<std::endl;
}
bool leg_control::motor_limit(std::vector<float>& kp,std::shared_ptr<motor> moto)//限制力矩
{
    bool start=true;
    if ((leg_pose[moto->_n]>moto->max_pose-0.1||leg_pose[moto->_n]<moto->min_pose+0.1)&&kp[0]>0)
    {
        // std::cout<<"motor"<<this->_leg_name<<moto->_n<<"出现"<<leg_pose[moto->_n]<<"\t"<<moto->max_pose-0.1
        // <<"\t"<<leg_pose[moto->_n]<<"\t"<<moto->min_pose+0.1<<std::endl;
        moto->set_motor_disable(3);
        start=false;
    }
    return start;
}
void leg_control::leg_init(){
    for (size_t i = 0; i < 100; i++)
    {
        set_leg_disable(5);
        _leg_motor0->start_pose=leg_data[0].Pos;
        _leg_motor1->start_pose=leg_data[1].Pos;
        _leg_motor2->start_pose=leg_data[2].Pos;
    }
    _leg_motor0->set_limt();
    _leg_motor1->set_limt();
    _leg_motor2->set_limt();
}