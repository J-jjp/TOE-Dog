
#include <MNN/Interpreter.hpp>
#include <MNN/Tensor.hpp>
#include <sstream>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
class rl_Inference
{
public:
    rl_Inference(std::string& modelPath, double dt, bool debugPrint = false);
    ~rl_Inference();
    void initBuffer();
    void resetNode();
    void advanceNNsync(const float observation[], float action_cmd[]);
protected:
    std::string prefix_path;
    float* currentActionPtr = nullptr;
    float* lastActionPtr = nullptr;
    std::shared_ptr<MNN::Interpreter> _net = nullptr;
    MNN::Session* _session = nullptr;
    MNN::Tensor* obs_mnn = nullptr;
    MNN::Tensor* act_mnn = nullptr;
    int obs_dim;
    bool debugPrint_, if_reset;
    int llc_step = 0;
};
