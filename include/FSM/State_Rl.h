/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef State_RL_H
#define State_RL_H

#include "FSM/FSMState.h"
#include "control/rl_Inference.h"
#define Num_dof 12
#define N_scan_Loco 187
#define N_priv_latent_Loco  4 + 1 + 12 + 12 + 12 + 6 + 1 + 4 + 1 - 3 + 4
#define N_proprio_Loco  45  
#define History_len_Loco 10
#define Num_observations_Loco  (N_proprio_Loco + N_priv_latent_Loco + N_scan_Loco + (History_len_Loco * N_proprio_Loco))

#define obs_scales_lin_vel 2
#define obs_scales_ang_vel 0.25
#define obs_scales_quat 1
#define obs_scales_dof_pos 1
#define obs_scales_dof_vel 0.05
// -----------------------------Walk_these_ways---------------------------------
#define NUM_DOFS 12
#define CMD_DIM 15
#define OBS_DIM 70
#define NUM_OBS_IN_OBS_HISTORY 30
#define ACTION_HISTORY_NUMS 7
// -----------------------------mast---------------------------------
#define N_proprio_mast  48
#define N_priv_latent_mast  4 + 1 + 12 + 12 + 12 + 6 + 1 + 4 + 1 - 3 + 3 - 3 + 4 - 7
#define Num_observations_mast  (N_proprio_mast + N_priv_latent_mast + N_scan_Loco + (History_len_Loco * N_proprio_mast))
// -----------------------------legged---------------------------------
#define N_proprio_legged  48
#define Num_observations_legged 48
class State_Rl : public FSMState{
public:
    State_Rl(CtrlComponents *ctrlComp);
    ~State_Rl(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();
    void mnnInference_Loco();
    void stateMachine_Loco();
    void mnnInference_mast();
    void stateMachine_mast();
    void mnnInference_legged();
    void stateMachine_legged();
    void mnnInference_Walk();
    void stateMachine_Walk();
    void getCurrentObservation_Walk();
    Eigen::Vector3d quat_rotate_inverse(const Eigen::Vector4d& q, const Eigen::Vector3d& v); 
    void mobRun();
    std::vector<float> default_dof_pos={0.1,0.8,-1.5 ,-0.1,0.8,-1.5,0.1,1,-1.5, -0.1,1.,-1.5};//#默认角度需要与isacc一致
    std::vector<float> default_dof_pos_mast={-0.1,0.8,-1.5 ,0.1,0.8,-1.5,-0.1,1,-1.5, 0.1,1.,-1.5};//#默认角度需要与isacc一致
    Vec3 quaternion_to_euler_array(Vec4 quat);
    std::shared_ptr<rl_Inference> rlptr = nullptr;
    std::shared_ptr<rl_Inference> adaptationNetPtr = nullptr;


    float last_lowCmd[Num_dof];
    float action_history[History_len_Loco*Num_dof]; //使用六帧前的数据
    float obs_history[1][History_len_Loco*N_proprio_Loco];
    float policy_input[1][Num_observations_Loco];
    float obs[1][N_proprio_Loco];
    float action_cmd[1][Num_dof];
    // -----------------------walk_these_ways------------------------
    float adaptation_output[2];
    float obs_history_with_adaptation[NUM_OBS_IN_OBS_HISTORY*OBS_DIM+2];
    bool walk_these_ways=false;
    float action_cmd_Walk[NUM_DOFS], last_action_cmd_Walk[NUM_DOFS];
    float obs_history_Walk[NUM_OBS_IN_OBS_HISTORY*OBS_DIM];
    float obs_buf_ [OBS_DIM];
    float proj_gravity[3];
    float mobCmd_[CMD_DIM];   //command
    float gait_indices;
    // -----------------------mast------------------------
    float obs_mast[1][N_proprio_mast];
    float obs_history_mast[1][History_len_Loco*N_proprio_mast];
    float policy_input_mast[1][Num_observations_mast];
    float action_stateq_mast[Num_dof];
    float action_statedq_mast[Num_dof];
    float action_cmd_mast[1][Num_dof];
    float last_action_cmd_mast[1][Num_dof];
    // -----------------------legged------------------------
    float obs_legged[N_proprio_legged];
    float policy_input_legged[Num_observations_legged];
    float action_stateq_legged[Num_dof];
    float action_statedq_legged[Num_dof];
    float action_cmd_legged[Num_dof];
    float last_action_cmd_legged[Num_dof];
};

#endif  // FREESTAND_H