//
// Created by hero on 19-6-18.
//

#include "admin/Timer.h"

Timer::Timer() {
    start=record=getTickCount();
    timeCounter=0;
}

double Timer::getLoopTime() {
    double time=(getTickCount()-record)/getTickFrequency()*1000;    //ms
    record=getTickCount();
    ++timeCounter;
    return time;
}

double Timer::averageTime() {
    double averageTime=(getTickCount()-start)/getTickFrequency()*1000/timeCounter;
    return averageTime;
}
string Timer::getDate() {

}