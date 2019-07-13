//
// Created by hero on 19-6-17.
//

#ifndef DRONEVISION_SETTING_H
#define DRONEVISION_SETTING_H

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class Setting {
public:
    explicit Setting(string settingParamPath);

    string videoPath="../others/video/";
    string picturePath="../others/picture/";
    string serialName="/dev/ttyUSB0";

    char key=' ';
    int detectMode=0;
    int showOrigin=0;
    int showGray=0;
    int showBinary=0;
    int showLEDbar=0;
    int showDraw=0;       //在图像上画出标识信息
    int printTxMsg=0;     //cout 输出接收到的串口信息
    int printRxMsg=0;     //cout 输出接收到的串口信息
    int printArmorPos=0;  //cout 输出装甲三维位置信息
    int printLoopTime=1;
    int droneWaitkey=0;
    int readOneFrame=0;
    int saveVideo=0;
    int writerFlag=0;
    int savePic=0;
};


#endif //DRONEVISION_SETTING_H
