/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef WIRELESSHANDLE_H
#define WIRELESSHANDLE_H

#include "interface/CmdPanel.h"
#include <unistd.h>  
#include <string.h>  
#include <sys/types.h>  
#include <sys/stat.h>  
#include <fcntl.h>  
#include <errno.h>  
#include <linux/input.h>  
#include <linux/joystick.h>  
#include "interface/CmdPanel.h"
#include "common/mathTools.h"
#include <thread>
#include <atomic>
#define XBOX_TYPE_BUTTON    0x01  
#define XBOX_TYPE_AXIS      0x02  

#define XBOX_BUTTON_A       0x00  
#define XBOX_BUTTON_B       0x01  
#define XBOX_BUTTON_X       0x02  
#define XBOX_BUTTON_Y       0x03  
#define XBOX_BUTTON_RB      0x05  

#define XBOX_BUTTON_ON      0x01  
#define XBOX_BUTTON_OFF     0x00  

#define XBOX_AXIS_LX        0x00    /* 左摇杆X轴 */  
#define XBOX_AXIS_LY        0x01    /* 左摇杆Y轴 */  
#define XBOX_AXIS_RX        0x03    /* 右摇杆X轴 */  
#define XBOX_AXIS_RY        0x04    /* 右摇杆Y轴 */  

#define XBOX__MIN       -32767  
#define XBOX__MAX       32767  

class WirelessHandle : public CmdPanel{
public:
    typedef struct xbox_map  
    {  
    int     a;  
    int     b;  
    int     x;  
    int     y;  
    int     lb;  
    int     rb;  

    int     lx;  
    int     ly;  
    int     rx;  
    int     ry;

    }xbox_map_t;
    WirelessHandle();
    ~WirelessHandle();
    void read_joy();
    void changeValue();
    UserCommand checkCmd(UserCommand userCommand);
    int xbox_map_read(int xbox_fd, xbox_map_t *map);
    float normalize_int_to_float(int value);
private:
    int xbox_fd ;  
    xbox_map_t map;  
    int len, type;  
    int axis_value, button_value;  
    int number_of_axis, number_of_buttons ;
    std::thread read_thread_;
    std::atomic<bool> reading_{false};
};

#endif  // WIRELESSHANDLE_H