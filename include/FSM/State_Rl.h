/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef State_RL_H
#define State_RL_H

#include "FSM/FSMState.h"
#include "control/rl_Inference.h"
#define Num_dof 12
#define N_scan_mast 187
#define N_priv_latent_mast  4 + 1 + 12 + 12 + 12 + 6 + 1 + 4 + 1 - 3 + 4
#define N_proprio_mast  45  
#define History_len_mast 10
#define Num_observations_mast  (N_proprio_mast + N_priv_latent_mast + N_scan_mast + (History_len_mast * N_proprio_mast))

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
// -----------------------------Loco---------------------------------
#define N_proprio_Loco  48
#define N_scan_Loco 187
#define History_len_Loco 10
#define N_priv_latent_Loco  4 + 1 + 12 + 12 + 12 + 6 + 1 + 4 + 1 - 3 + 3 - 3 + 4 - 7
#define Num_observations_Loco  (N_proprio_Loco + N_priv_latent_Loco + N_scan_Loco + (History_len_Loco * N_proprio_Loco))
// -----------------------------legged---------------------------------
#define N_proprio_legged  42
#define Num_observations_legged 42*10
#define Num_encoder_legged 32
#define History_len_legged 9
// -----------------------------que---------------------------------
#define N_proprio_que  42
#define Num_observations_que  42*15
#define Num_encoder_que  32
#define History_len_que  15
// -----------------------------amp--------------------------------
#define N_proprio_amp  42
#define Num_observations_amp 42*10
#define Num_encoder_amp 32
#define History_len_amp 1
#define PI 3.141592653589793f

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
    void mnnInference_que();
    void stateMachine_que();
    void mnnInference_amp();
    void stateMachine_amp();
    void mnnInference_mujoco();
    void stateMachine_mujoco();
    void mnnInference_Walk();
    void stateMachine_Walk();
    void getCurrentObservation_Walk();
    void mnnInference_backflip();
    void stateMachine_backflip();
    void Pose_transformation();
    void time_zaro();
    void test_motor();
    void speed_limit();
    float gear_ratio;
    int time_rl; 
    Eigen::Vector3d quat_rotate_inverse(const Eigen::Vector4d& q, const Eigen::Vector3d& v); 
    void mobRun();
    std::vector<float> default_dof_pos={0.1,0.8,-1.5 ,-0.1,0.8,-1.5,0.1,1,-1.5, -0.1,1.,-1.5};//#默认角度需要与isacc一致
    std::vector<float> default_dof_pos_amp={0.,0.9,-1.8 ,-0.,0.9,-1.8, 0.,0.9,-1.8, -0.,0.9,-1.8};//#默认角度需要与isacc一致
    Vec3 quaternion_to_euler_array(Vec4 quat);
    std::shared_ptr<rl_Inference> rlptr = nullptr;
    std::shared_ptr<rl_Inference> adaptationNetPtr = nullptr;
    int modle;
    float sin_counter;
    float output_angle_c;
    bool shape_control;
    bool modle_control;
    float last_lowCmd[Num_dof];
    float action_history[History_len_mast*Num_dof]; //使用六帧前的数据
    float obs_history[1][History_len_mast*N_proprio_mast];
    float policy_input[1][Num_observations_mast];
    float obs[1][N_proprio_mast];
    float action_cmd[1][Num_dof];
    // -----------------------walk_these_ways------------------------
    float adaptation_output[2];
    float obs_history_with_adaptation[NUM_OBS_IN_OBS_HISTORY*OBS_DIM+2];
    bool walk_these_ways=true;
    float action_cmd_Walk[NUM_DOFS], last_action_cmd_Walk[NUM_DOFS];
    float obs_history_Walk[NUM_OBS_IN_OBS_HISTORY*OBS_DIM];
    float obs_buf_ [OBS_DIM];
    float proj_gravity[3];
    float mobCmd_[CMD_DIM];   //command
    float gait_indices;
    // -----------------------loco------------------------
    float obs_Loco[N_proprio_Loco];
    float obs_history_Loco[History_len_Loco*N_proprio_Loco];
    float policy_input_Loco[Num_observations_Loco];
    float action_stateq_Loco[Num_dof];
    float action_statedq_Loco[Num_dof];
    float action_cmd_Loco[Num_dof];
    float last_action_cmd_Loco[Num_dof];
    float priv_latent[N_priv_latent_Loco];
    // -----------------------legged------------------------
    float obs_legged[N_proprio_legged];

    float encoder_input_legged[Num_observations_legged];
    float encoder_output_legged[Num_encoder_legged];
    float obs_history_legged[History_len_legged*N_proprio_legged];
    float policy_input_legged[Num_observations_legged+Num_encoder_legged];

    float action_cmd_legged[Num_dof];
    float last_action_cmd_legged[Num_dof];
    // -----------------------que------------------------
    float obs_que[N_proprio_que];

    float encoder_input_que[Num_observations_que];
    float encoder_output_que[Num_encoder_que];
    float obs_history_que[History_len_que*N_proprio_que];
    float policy_input_que[Num_observations_que+Num_encoder_que];

    float action_cmd_que[Num_dof];
    float last_action_cmd_que[Num_dof];
    // -----------------------amp------------------------
    float obs_amp[N_proprio_amp];

    float encoder_input_amp[Num_observations_amp];
    float encoder_output_amp[Num_encoder_amp];
    float obs_history_amp[History_len_amp*N_proprio_amp];
    float policy_input_amp[Num_observations_amp+Num_encoder_amp];

    float action_cmd_amp[Num_dof];
    float last_action_cmd_amp[Num_dof];
        // -----------------------backflip------------------------
    float obs_backflip[60];
    // float policy_input_backflip[60];
    float action_stateq_backflip[Num_dof];
    float action_statedq_backflip[Num_dof];
    float action_cmd_backflip[Num_dof];
    float last_action_cmd_backflip[Num_dof];
    float last_last_action_cmd_backflip[Num_dof];
    int backflip_time=0;
};

#endif  // FREESTAND_H