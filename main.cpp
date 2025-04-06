
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

#include <GLFW/glfw3.h>
#include <interface/IOMujoco.h>
#include <control/CtrlComponents.h>
#include <FSM/FSM.h>
#include <string>
#include <iostream>
#include <control/ControlFrame.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;
float kp_all = 30;
float kd_all = 0.75;
bool running = true;

std::vector<mjtNum> get_sensor_data(const mjModel *model, const mjData *data, const std::string &sensor_name)
{
  int sensor_id = mj_name2id(model, mjOBJ_SENSOR, sensor_name.c_str());
  if (sensor_id == -1)
  {
    std::cout << "no found sensor" << std::endl;
    return std::vector<mjtNum>();
  }
  int data_pos = 0;
  for (int i = 0; i < sensor_id; i++)
  {
    data_pos += model->sensor_dim[i];
  }
  std::vector<mjtNum> sensor_data(model->sensor_dim[sensor_id]);
  for (int i = 0; i < sensor_data.size(); i++)
  {
    sensor_data[i] = data->sensordata[data_pos + i];
  }
  return sensor_data;
}
// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mj_forward(m, d);
  }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  // update button state
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


// main function
int main(int argc,char** argv) {
  mjvCamera *cam1=&cam;                      // abstract camera
  mjvOption *opt1=&opt;                      // visualization options
  mjvScene *scn1=&scn;                       // abstract scene
  mjrContext *con1=&con;                     // custom GPU context
  ros::init(argc, argv, "array_publisher");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("array_data", 10);
  ros::Rate rate(50);
  // load and compile model
  char error[1000] = "Could not load binary model";
  m = mj_loadXML("../robot/TOE_dog/xml/scene.xml", 0, error, 1000);
  if (!m) {
    mju_error("Load model error: %s", error);
  }

  // make data
  d = mj_makeData(m);

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }
  // create window, make OpenGL context current, request v-sync
  GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);


    IOInterface *ioInter;
    CtrlPlatform ctrlPlat;
    int x=0;
    float _percent=0;
    float _duration=1000;
    ioInter = new IOMujoco(d,m);
    std::vector<float> start_pose(12);
    ctrlPlat = CtrlPlatform::Mujoco;
    CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
    ctrlComp->ctrlPlatform = ctrlPlat;
    ctrlComp->dt = 0.0025; // run at 400hz  控制周期       
    ctrlComp->running = &running;  //机器人控制的状态  运行 or 不运行
    ctrlComp->robotModel = new A1Robot();
    ControlFrame ctrlFrame(ctrlComp);
    int time=0;
  while (ros::ok() && !glfwWindowShouldClose(window)) {
      time++;
      mjtNum simstart = d->time;
      float kp=0;
      float kd=0;
      // m->opt.timestep = 0.001;
      int count=0;
      ctrlFrame.run();
  
      std_msgs::Float32MultiArray array_msg;

      // 填充数据（示例：12 个 float）
      std::vector<float> data(12);
      for (size_t i = 0; i < 12; i++)
      {
        data[i]=ctrlComp->lowState->imu.gyroscope[i];
      }

      
      // 设置数据到消息
      array_msg.data = data;

      // 发布消息
      pub.publish(array_msg);

      ROS_INFO("Published array: [%f, %f, %f, ...]", data[0], data[1], data[2]);

      
    
      while (d->time - simstart < 1.0/60.0) {


        mj_step(m, d);
        count++;
      }
      
    // get framebuffer viewport
      mjrRect viewport = {0, 0, 0, 0};
      glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

      // update scene and render
      mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
      mjr_render(viewport, &scn, &con);

      // swap OpenGL buffers (blocking call due to v-sync)
      glfwSwapBuffers(window);

      // process pending GUI events, call GLFW callbacks
      glfwPollEvents();
      rate.sleep();

  }

  //free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);
  return 1;
}
