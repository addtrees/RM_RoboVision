//
// Created by Nicapoet on 18-11-20.
//

#include <IndustrialCamera/CameraDefine.h>
#include <string>
#include "../include/IndustrialCamera/MindVisionCamera.h"

MindVisionCamera::MindVisionCamera() : is_read_video(false), is_save_video(false) {
    int camera_counts = 1;
    CameraSdkInit(1);
    int status = CameraEnumerateDevice(&tCameraEnumList, &camera_counts);

    if (status) {
        std::cout << "camera enum fail,error code=" << status << std::endl;
        exit(-1);
    }

    if (!camera_counts) {
        std::cout << "camera_disconnect" << std::endl;
        exit(-2);
    }

    status = CameraInit(&tCameraEnumList, -1, -1, &h_camera);
    if (status) {
        std::cout << "camera init fail" << std::endl;
        exit(-3);
    }

    CameraGetCapability(h_camera, &tCapability);
    g_pRgbBuffer = (unsigned char *) malloc(tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3);
    //set exposure time by proto file
    setExposureTime(10);
    //////////////////////////////////////////////////////
    //set Image Output Setting
    CameraGetImageResolution(h_camera, &tImageResolution);
    tImageResolution.iIndex = 0xff;
    //[1]set outputimage size
    tImageResolution.iWidth = 800;
    tImageResolution.iHeight = 600;
    tImageResolution.iWidthFOV = 800;
    tImageResolution.iHeightFOV = 600;
    //[2]set outputimage's offset in the full image
    tImageResolution.iHOffsetFOV = 240;
    tImageResolution.iVOffsetFOV = 212;
    CameraSetImageResolution(h_camera, &tImageResolution);
    CameraPlay(h_camera);

    CameraSetIspOutFormat(h_camera, CAMERA_MEDIA_TYPE_BGR8);
    CameraSetAeState(h_camera, FALSE);
    CameraSetFrameSpeed(h_camera, 1);
}

MindVisionCamera::~MindVisionCamera() {
    if (is_read_video) {
        CameraUnInit(h_camera);
        free(g_pRgbBuffer);
    } else {
        local_video.release();
    }
    if (is_save_video) {
        writer.release();
    }
}

void MindVisionCamera::setExposureTime(double ExposureTime_ms) {
    CameraSetExposureTime(h_camera, ExposureTime_ms * 1000);
}

void MindVisionCamera::setResolution(cv::Size roi, cv::Size offset)
{
    tImageResolution.iIndex = 0xff;
    tImageResolution.iWidth = roi.width;
    tImageResolution.iHeight = roi.height;
    tImageResolution.iWidthFOV = roi.width;
    tImageResolution.iHeightFOV = roi.height;
    tImageResolution.iHOffsetFOV = ((1280 - roi.width) >> 1) + offset.width;
    tImageResolution.iVOffsetFOV = ((1024 - roi.height) >> 1) + offset.height;
    CameraSetImageResolution(h_camera, &tImageResolution);
}

bool MindVisionCamera::read(cv::Mat &frame) {
    IplImage *iplImage = NULL;
    tSdkFrameHead sFrameInfo;
    BYTE *pbyBuffer;
    if (CameraGetImageBuffer(h_camera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS) {
        CameraImageProcess(h_camera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);
        if (iplImage) {
            cvReleaseImageHeader(&iplImage);
        }
        iplImage = cvCreateImageHeader(cvSize(sFrameInfo.iWidth, sFrameInfo.iHeight), IPL_DEPTH_8U, 3);
        cvSetData(iplImage, g_pRgbBuffer, sFrameInfo.iWidth * 3);

        frame = cv::cvarrToMat(iplImage);
        if (is_save_video) { writer << frame; }
        //std::cout<<"cols="<<frame.cols<<" rows="<<frame.rows<<std::endl;
        CameraReleaseImageBuffer(h_camera, pbyBuffer);
        return true;
    }
}

void MindVisionCamera::toSaveVideo(char **file_name) {
    if (file_name[1] != NULL && file_name[1][0] != '-') {
        std::cout << "to save video: open  " << file_name << std::endl;
    } else {
        std::cout << "to save video: close" << std::endl;
        return;
    }
    is_save_video = true;
    std::string video_file_path(file_name[1]);
    writer.open(
            "../others/" + video_file_path + ".avi",
            CV_FOURCC('M', 'J', 'P', 'G'),
            25.0,
            cv::Size(tImageResolution.iWidth, tImageResolution.iHeight)
    );
}

void MindVisionCamera::toReadLocalVideo(char **file_name) {
    if (file_name[1] != NULL && file_name[2] != NULL) {
        std::string file_n(file_name[2]);
        std::cout << "Read Video: ../others/" + file_n << std::endl;
        is_read_video = true;
        local_video.open("../others/" + file_n + ".avi");
    }

}
