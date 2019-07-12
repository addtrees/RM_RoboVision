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
                    char(254),};
    int TxFlag=0;
    int RxFlag=0;
    int STOP=0;
    Serial *serial;
    bool debugMode_2_thread();
    void* msgDeal();
    void* droneAttack();
    void* debugMode();
    void calibration();
    void trainNumberRecognizeModel();
};



#endif //DRONEVISION_MODEACHIEVE_H
