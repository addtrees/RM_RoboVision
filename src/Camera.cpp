//
// Created by hero on 19-5-31.
//

#include "Camera.h"

Camera::Camera(string cameraParamPath) {
    cout<<"Union Camera Param Path:"<<cameraParamPath;
    int isVideo;
    int cameraType;
    string videoPath;
    cv::FileStorage fs(cameraParamPath,cv::FileStorage::READ);
    fs.isOpened()?cout<<"    opened."<<endl:cout<<"    open failed."<<endl;
    if(!fs.isOpened())
        exit(-1);
    fs["isVideo"]>>isVideo;
    fs["videoPath"]>>videoPath;
    fs["cameraType"]>>cameraType;
    fs["exposureTime"]>>exposureTime;
    fs["imgSize"]>>imgSize;
    fs["offset"]>>offset;
    cout<<"isVideo   : "<< static_cast<bool>(isVideo)<<endl
        <<"cameraType: "<<cameraType<<endl
        <<"videoPath : "<<videoPath<<endl<<endl;
    if(isVideo) {
         isIndustrialCamera = false;
        camera.normalCamera = new cv::VideoCapture;
        camera.normalCamera->open(videoPath);
        if (!camera.normalCamera->isOpened()){
            cout << "cannot open video " << videoPath << endl;
            exit(-2);
        }
    }else{
        if(cameraType<0){
            isIndustrialCamera=true;
            camera.industrialcamera=new MindVisionCamera();
            camera.industrialcamera->setExposureTime(exposureTime);
            cout<<"expose time:"<<exposureTime<<endl;
            camera.industrialcamera->setResolution(imgSize,offset);//设置ROI大小和方向偏置
        }else{
            isIndustrialCamera=false;
            camera.normalCamera=new VideoCapture(cameraType);
            if(!camera.normalCamera->isOpened()){
                cout<<"cannot open USB camera."<<endl;
                exit(-4);
            }
        }
    }
}

bool Camera::read(cv::Mat &frame) {
    if(isIndustrialCamera){
        bool flag=camera.industrialcamera->read(frame);
/*******************工业相机防掉线*************************/
        while(frame.empty()){
            delete camera.industrialcamera;
            camera.industrialcamera=new MindVisionCamera();
            flag=camera.industrialcamera->read(frame);
        }
        return flag;
    }
    else{
        bool flag=camera.normalCamera->read(frame);
        return flag;
    }
}