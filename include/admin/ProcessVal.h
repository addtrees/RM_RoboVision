//
// Created by hero on 19-6-18.
//

#ifndef DRONEVISION_PROCESSVAL_H
#define DRONEVISION_PROCESSVAL_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "ArmorDetect.h"
using namespace std;
using namespace cv;

class ProcessVal {
public:
    void update(ArmorDetect &detector);
    Mat src;
    Mat gray;
    Mat binary;
    Mat sketch;
    VideoWriter videoWriter;

};


#endif //DRONEVISION_PROCESSVAL_H
