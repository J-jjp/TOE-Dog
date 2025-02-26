
#include <../MNN/Interpreter.hpp>
#include <../MNN/Tensor.hpp>
#include <sstream>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
class rl_Inference
{
public:
    rl_Inference(std::string& modelPath);
    ~rl_Inference();
    void initBuffer();
    void resetNode();
    void advanceNNsync(float (*observation)[736], float (*action_cmd)[12]);
    void advanceNNsync_Loco(float (*observation)[762], float (*action_cmd)[12]);
    void advanceNNsync_legged(float (*observation)[235], float (*action_cmd)[12]);
    void advanceNNsync_Walk(const float observation[], float action_cmd[]);
protected:
    std::string prefix_path;
    float* currentActionPtr = nullptr;
    float* lastActionPtr = nullptr;
    std::shared_ptr<MNN::Interpreter> _net = nullptr;
    MNN::Session* _session = nullptr;
    MNN::Tensor* obs_mnn = nullptr;
    MNN::Tensor* act_mnn = nullptr;
    int obs_dim,act_dim;
    bool debugPrint_, if_reset;
    int llc_step = 0;
};
