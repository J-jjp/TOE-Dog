#ifndef IOREAL_H
#define IOREAL_H

#include "control/leg_control.h"
#include "interface/KeyBoard.h"
#include "interface/WirelessHandle.h"
#include "interface/IOInterface.h"
#include <string>
class IOReal : public IOInterface{
public:
    IOReal(/* args */){
        std::cout<<"generate interfaces"<<std::endl;
        if (1)
        {
            cmdPanel = new WirelessHandle();
        }
        else{
            cmdPanel = new KeyBoard();
        }
        FL_leg = std::make_shared<leg_control>("FL", "/dev/ttyUSB0",FL_dir);
        FR_leg = std::make_shared<leg_control>("FR", "/dev/ttyUSB1",FR_dir);
        RL_leg = std::make_shared<leg_control>("RL", "/dev/ttyUSB2",RL_dir);
        RR_leg = std::make_shared<leg_control>("RR", "/dev/ttyUSB3",RR_dir);
        legs.push_back(FL_leg);
        legs.push_back(FR_leg);
        legs.push_back(RL_leg);
        legs.push_back(RR_leg);
    };
    void sendRecv(LowlevelCmd *cmd, LowlevelState *state);
    void send(LowlevelCmd *cmd);
    void recv(LowlevelState *state);
    void prit_state();
    ~IOReal();
public:
    std::vector<int> FL_dir={1,1,1};
    std::vector<int> FR_dir={1,-1,-1};
    std::vector<int> RL_dir={-1,1,1};
    std::vector<int> RR_dir={-1,-1,-1};
    std::vector<std::shared_ptr<leg_control>> legs;
    std::shared_ptr<leg_control>  FL_leg;
    std::shared_ptr<leg_control>  FR_leg;
    std::shared_ptr<leg_control>  RL_leg;
    std::shared_ptr<leg_control>  RR_leg;
};

#endif