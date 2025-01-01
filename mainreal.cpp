
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdio>
#include <cstring>
#include "interface/IOReal.h"
#include <control/CtrlComponents.h>
#include <FSM/FSM.h>
#include <string>
#include <iostream>
#include <control/ControlFrame.h>
// std::vector<float> default_dof_pos={0.1,0.8,-1.5 ,-0.1,0.8,-1.5,0.1,1,-1.5, -0.1,1.,-1.5};
std::vector<float> default_dof_pos={0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.};//#默认角度需要与isacc一致
bool running = true;
void setProcessScheduler()
{
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}
// main function
int main(int argc, const char** argv) {
    LowlevelCmd *lowCmd = new LowlevelCmd();
    LowlevelState *lowState = new LowlevelState();
    setProcessScheduler();
    IOInterface *ioInter;
    CtrlPlatform ctrlPlat;
    ioInter = new IOReal();
    ctrlPlat = CtrlPlatform::REALROBOT;
    CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->dt = 0.0025; 
    ctrlComp->running = &running;
    ctrlComp->robotModel = new ToeRobot();

    ControlFrame ctrlFrame(ctrlComp);
  while (1) {
        ctrlFrame.run();
        // for (size_t i = 0; i < 12; i++)
        // {
        //   lowCmd->motorCmd[i].q=default_dof_pos[i];
        // }
        
        // ioInter->sendRecv(lowCmd,lowState);

  }
  return 1;
}
