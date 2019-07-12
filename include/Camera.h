//
// Created by hero on 19-5-31.
//
//
#ifndef DRONEVISION_CAMERA_H
#define DRONEVISION_CAMERA_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "MindVisionCamera.h"
#include "CameraDefine.h"
#include "CameraStatus.h"
#include "CameraApi.h"

using namespace std;
using namespace cv;

struct unionCamera{
    MindVisionCamera *industrialcamera;
    VideoCapture *normalCamera;
};
class Camera {
public:
    explicit Camera(string cameraParamPath);
    unionCamera camera;
    bool read(cv::Mat &frame);
    bool isIndustrialCamera;
    Size imgSize;
    Point offset;
    int exposureTime=3;
};


#endif //DRONEVISION_CAMERA_H
