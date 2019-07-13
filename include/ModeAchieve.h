//
// Created by hero on 19-5-31.
//

#ifndef DRONEVISION_MODEACHIEVE_H
#define DRONEVISION_MODEACHIEVE_H

#include <thread>
#include "ProcessStep.h"

using namespace std;
class ModeAchieve {
public:
    ModeAchieve();
    char TxMsg[10];
    char RxMsg[10]={char(255),
                    char(178),
                    char(0)  ,
                    char(0)  ,
                    char(0)  ,
                    char(0)  ,
                    char(0)  ,
                    char(0)  ,
                    char(0)  ,
                    char(254)};
    int TxFlag=0;  //send message flag
    int RxFlag=0;  //recieve message flag
    int STOP=0;    //control the thread stop
    int readingImage=0;//read image flag
    int waitKeyFlag=1;
    int readOneFrame=0;
    Mat frame;
    Serial *serial;
    Camera *camera;
    bool debugMode_2_thread();
    bool* readImage();
    void* msgDeal();
    void* droneAttack();
    void* debugMode();
    void calibration();
    void trainNumberRecognizeModel();
};



#endif //DRONEVISION_MODEACHIEVE_H
