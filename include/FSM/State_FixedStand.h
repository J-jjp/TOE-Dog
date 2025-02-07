#ifndef FIXEDSTAND_H
#define FIXEDSTAND_H

#include "FSMState.h"

class State_FixedStand : public FSMState{
public:
    State_FixedStand(CtrlComponents *ctrlComp);
    ~State_FixedStand(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:

    float _targetPos[12] = {-0.1,0.8,-1.5 ,0.1,0.8,-1.5,-0.1,1,-1.5, 0.1,1.,-1.5};
    float _defpos[12] = {0.,0.,0. ,0.,0.,0,0.,0,0, 0.,0.,0};
    float _startPos[12];
    float _duration = 100;   //steps
    float _percent = 0;       //%
    bool jump = false;
};
#endif
