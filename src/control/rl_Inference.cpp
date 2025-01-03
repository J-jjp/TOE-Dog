#include "control/rl_Inference.h"
rl_Inference::rl_Inference(std::string& modelPath, double dt, bool debugPrint = false)
    :prefix_path(modelPath)
{   
  std::string net_path = prefix_path ;
    _net = std::shared_ptr<MNN::Interpreter> (MNN::Interpreter::createFromFile(net_path.c_str()));
    MNN::ScheduleConfig config;
    config.numThread = 2;
    _session = _net->createSession(config);
}

rl_Inference::~rl_Inference()
{
    _net->releaseModel();
    _net->releaseSession(_session);
    delete[] currentActionPtr;
    delete[] lastActionPtr;
}
void rl_Inference::initBuffer()
{
  obs_mnn = _net->getSessionInput(_session, nullptr);
  act_mnn = _net->getSessionOutput(_session, nullptr);

  obs_dim = obs_mnn->shape().back();
  std::cout << "obs_dim: " << obs_dim << std::endl;

  currentActionPtr = new float[act_mnn->shape().back()]();
  lastActionPtr = new float[act_mnn->shape().back()]();
  std::cout << "mlp net init finished" << std::endl;
}
void rl_Inference::resetNode()
{
  if_reset = true;
  llc_step = 0;
}
void rl_Inference::advanceNNsync(float (*observation)[45], float (*action_cmd)[12])
{

  // std::cout<<obs_mnn->size()<<std::endl;
  memcpy(obs_mnn->host<float>() , observation, obs_mnn->size());

  _net->runSession(_session);

  memcpy(action_cmd, act_mnn->host<float>(), act_mnn->size());

}