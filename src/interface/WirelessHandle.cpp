#include "interface/WirelessHandle.h"



WirelessHandle::WirelessHandle(){
    memset(&map, 0, sizeof(xbox_map_t));  
    xbox_fd = open("/dev/input/js0", O_RDONLY);  
    if (xbox_fd < 0)  
    {  
        perror("open");  
        std::cout<<"open error"<<std::endl;
    }
    reading_ = true;
    read_thread_ = std::thread(&WirelessHandle::read_joy, this);
}
WirelessHandle::~WirelessHandle(){
    close(xbox_fd);
    reading_ = false;
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
}
void WirelessHandle::read_joy(){
    while (1)
    {
        len = xbox_map_read(xbox_fd, &map);
        if (len < 0)  
        {  
            usleep(10*1000);  
            continue;  
        } 
        if (map.rb == 1)
        {
            std::cout<<"换模型";
            userCmd=checkCmd(userCmd);
        }
        changeValue();

        // printf("\rTime:%8d A:%d B:%d X:%d Y:%d LB:%d RB:%d LX:%-6d LY:%-6d RX:%-6d RY:%-6d",  
        //         map.a, map.b, map.x, map.y, map.lb, map.rb,   
        //          map.lx, map.ly, map.rx, map.ry);
        std::cout<<"A"<<map.a<<"B"<<map.b<<"X"<<map.x<<"Y"<<map.y<<"LX"<<map.lx<<"LY"<<map.ly<<"RB"<<map.rb
        <<"RX"<<map.rx<<"RY"<<map.ry<<std::endl;
        fflush(stdout);  
    }
}


int WirelessHandle::xbox_map_read(int xbox_fd, xbox_map_t *map)  
{  
    int len, type, number, value;  
    struct js_event js;  
    len = read(xbox_fd, &js, sizeof(struct js_event));  
    if (len < 1)  
    {  
        perror("read");  
        return -1;  
    }  

    type = js.type;  
    number = js.number;  
    value = js.value;  


    if (type == JS_EVENT_BUTTON)  
    {  
        switch (number)  
        {  
            case XBOX_BUTTON_A:  
                map->a = value;  
                break;  

            case XBOX_BUTTON_B:  
                map->b = value;  
                break;  

            case XBOX_BUTTON_X:  
                map->x = value;  
                break;  

            case XBOX_BUTTON_Y:  
                map->y = value;  
                break;  

            case XBOX_BUTTON_RB:  
                map->rb = value;  
                break;  
            default:  
                break;  
        }  
    }  
    else if (type == JS_EVENT_AXIS)  
    {  
        switch(number)  
        {  
            case XBOX_AXIS_LX:  
                map->lx = value;  
                break;  

            case XBOX_AXIS_LY:  
                map->ly = value;  
                break;  

            case XBOX_AXIS_RX:  
                map->rx = value;  
                break;  

            case XBOX_AXIS_RY:  
                map->ry = value;  
                break;  
            default:  
                break;  
        }  
    }  

    return len;  
}  


UserCommand WirelessHandle::checkCmd(UserCommand userCommand){
    if (map.a == 1)
    {
        return UserCommand::RL;
    }
    else if(map.b == 1){
        return UserCommand::FIXED;
    }
    else if(map.x == 1){
        return UserCommand::FREE;
    }
    else if(map.y == 1){
        return UserCommand::PASS;    
    }
    else{
        return userCommand;
    }
}

void WirelessHandle::changeValue(){
    float ly=normalize_int_to_float(map.ly);
    float lx=normalize_int_to_float(map.lx);
    float ry=normalize_int_to_float(map.ry);
    float rx=normalize_int_to_float(map.rx);
        // std::cout<<"A"<<map.a<<"B"<<map.b<<"X"<<map.x<<"Y"<<map.y<<"LX"<<map.lx<<"LY"<<map.ly<<"RB"<<map.rb
        // <<"RX"<<map.rx<<"RY"<<map.ry<<std::endl;
    userValue.ly = max<float>(-1.f,min<float>(ly, 1.f));
    userValue.lx = max<float>(-1.f,min<float>(lx, 1.f));
    userValue.ry = max<float>(-1.f,min<float>(ry, 1.f));
    userValue.rx = max<float>(-1.f,min<float>(rx, 1.f));

    userValue.lx = killZeroOffset(userValue.lx, 0.08);
    userValue.ly = killZeroOffset(userValue.ly, 0.08);
    userValue.rx = killZeroOffset(userValue.rx, 0.08);
    userValue.ry = killZeroOffset(userValue.ry, 0.08);
}
float WirelessHandle::normalize_int_to_float(int value){
    float range = static_cast<float>(XBOX__MAX - XBOX__MIN);
    // 计算归一化后的值
    return 2.0f * (value - XBOX__MIN) / range - 1.0f;
}