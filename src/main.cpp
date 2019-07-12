#include <thread>
#include <iostream>
#include "ModeAchieve.h"

#define NUM_THREAD 2

using namespace std;

int main(int argv,char **argc) {
    ModeAchieve god;
    if(argv==1)
        god.debugMode();//_2_thread
    else if(argv==2){
        switch (argc[1][0]){
            case 'f':
                god.droneAttack();  break;
            case 'c':
                god.calibration();  break;
            case 't':
                god.trainNumberRecognizeModel();  break;
            default:
                return -1;
        }
    }
    return 0;
}