//
// Created by hero on 19-6-13.
//

#ifndef DRONEVISION_PROCESSSTEP_H
#define DRONEVISION_PROCESSSTEP_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>
#include "Camera.h"
#include "Calibrate.h"
#include "ArmorDetect.h"
#include "admin/Timer.h"
#include "admin/Setting.h"
#include "admin/ProcessVal.h"
#include "AngleSolve.h"
#include "SpatialSolution.h"
#include "MindVisionCamera.h"
#include "Serial.h"

using namespace std;
using namespace cv;
//input an capture picture,output all armor's four corner's pixel position
void armorDetection(cv::Mat &frame,vector<cv::Point[4]> &corners);

//input all armor's four corners,output an 3xn matrix as those armor's real position information
void pnpSolution(vector<cv::Point[4]> corners,cv::Mat &positionInfo);

//according to the armor's real position information,calculating all target point
void ballisticSolution(cv::Mat positionInfo,int droneState[3],cv::Point &targetPoint);

//according to the armor's size,target position,make the best choice to shoot,this function will remove those useless objects
Point decision(ArmorDetect &detector,AngleSolve &angleSolver,int solveObj);

Point decisionX(ArmorDetect &detector,AngleSolve &angleSolver);

void write2Serial(AngleSolve &solve,Serial &serial,bool isDetected);

void printLoopTime(Timer &timer);

void averageTime(Timer &timer);

void printFPS(Timer &timer);

void averageFPS(Timer &timer);

void printDate(Timer &timer);

char droneWaitKey(Setting &setter);

void waitkeyAction(Setting &setter,ProcessVal &val);
#endif //DRONEVISION_PROCESSSTEP_H
