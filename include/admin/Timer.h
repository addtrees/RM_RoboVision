//
// Created by hero on 19-6-18.
//

#ifndef DRONEVISION_TIMER_H
#define DRONEVISION_TIMER_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <time.h>

using namespace std;
using namespace cv;

class Timer {
public:
    int64 record;       //记录上次时间
    int64 start;        //对象定义时间
    int64 timeCounter;  //记录循环次数
    Timer();
    double averageTime();
    double getLoopTime();
};


#endif //DRONEVISION_TIMER_H
