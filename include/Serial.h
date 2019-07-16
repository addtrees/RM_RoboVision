//
// Created by hero on 19-5-31.
//

#ifndef DRONEVISION_SERIALPORT_H
#define DRONEVISION_SERIALPORT_H



#include <string.h>
#include <stdio.h>      /*标准输入输出定义*/
#include <stdlib.h>     /*标准函数库定义*/
#include <unistd.h>     /*Unix 标准函数定义*/
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX 终端控制定义*/
#include <errno.h>      /*错误号定义*/
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class Serial {
public:
    explicit Serial(string serialParamPath);
    size_t readBuffer(char *inputBuffer,size_t bufferSize);
    size_t writeBuffer(char *outputBuffer,size_t bufferSize);
    bool readByte(char *inputData);
    bool writeByte(char *outputData);
    void closeSerial();
    int showMsgTogether=0;
    int showTxMsgInTime=0;
    int showRxMsgInTime=0;
    int inWar=0;
    int fd;
    string portName;
};

#endif //DRONEVISION_SERIALPORT_H
