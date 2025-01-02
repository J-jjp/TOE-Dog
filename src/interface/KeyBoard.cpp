/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "interface/KeyBoard.h"
#include <iostream>

KeyBoard::KeyBoard(){
    userCmd = UserCommand::NONE;
    userValue.setZero();

    tcgetattr( fileno( stdin ), &_oldSettings );
    _newSettings = _oldSettings;
    _newSettings.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr( fileno( stdin ), TCSANOW, &_newSettings );

    pthread_create(&_tid, NULL, runKeyBoard, (void*)this);
}

KeyBoard::~KeyBoard(){
    pthread_cancel(_tid);
    pthread_join(_tid, NULL);
    tcsetattr( fileno( stdin ), TCSANOW, &_oldSettings );
}

UserCommand KeyBoard::checkCmd(){
    switch (_c){
    case '1':
        return UserCommand::NONE;
    case '2':
        return UserCommand::FIXED;
    // case '3':
    //     return UserCommand::L2_X;
    case '4':
        return UserCommand::FREE;
    case '0':
        return UserCommand::PASS;
    // case '9':
    //     return UserCommand::L1_A;
    // case '8':
    //     return UserCommand::L1_Y;
    case ' ':
        userValue.setZero();
        return UserCommand::NONE;
    default:
        return UserCommand::NONE;
    }
}

void KeyBoard::changeValue(){
    switch (_c){
    case 'w':case 'W':
        userValue.ly = min<float>(userValue.ly+sensitivityLeft, 1.0);
        break;
    case 's':case 'S':
        userValue.ly = max<float>(userValue.ly-sensitivityLeft, -1.0);
        break;
    case 'd':case 'D':
        userValue.lx = min<float>(userValue.lx+sensitivityLeft, 1.0);
        break;
    case 'a':case 'A':
        userValue.lx = max<float>(userValue.lx-sensitivityLeft, -1.0);
        break;

    case 'i':case 'I':
        userValue.ry = min<float>(userValue.ry+sensitivityRight, 1.0);
        break;
    case 'k':case 'K':
        userValue.ry = max<float>(userValue.ry-sensitivityRight, -1.0);
        break;
    case 'l':case 'L':
        userValue.rx = min<float>(userValue.rx+sensitivityRight, 1.0);
        break;
    case 'j':case 'J':
        userValue.rx = max<float>(userValue.rx-sensitivityRight, -1.0);
        break;
    default:
        break;
    }
}

void* KeyBoard::runKeyBoard(void *arg){
    std::cout<<"create thread";
    ((KeyBoard*)arg)->run(NULL);
    return NULL;
}

void* KeyBoard::run(void *arg){
    while(1){
        // std::cout<<"wait for key";
        FD_ZERO(&set);
        FD_SET( fileno( stdin ), &set );

        res = select( fileno( stdin )+1, &set, NULL, NULL, NULL);

        if(res > 0){
            ret = read( fileno( stdin ), &_c, 1 );
            userCmd = checkCmd(); //这里检查一下_c  每个按键应该会对应一个数字  查看那个按键按下
            if(userCmd == UserCommand::NONE) //如果不是'0','1'...... 就检查是不是 'w','s'......
                changeValue();
            _c = '\0';  //清空
        }
        usleep(1000);
    }
    return NULL;
}