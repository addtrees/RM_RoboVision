//
// Created by hero on 19-5-31.
//

#ifndef DRONEVISION_CALIBRATION_H
#define DRONEVISION_CALIBRATION_H

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

class Calibrate {
public:
    explicit Calibrate(string calibrationParam);
    void findChessBoardCorners(string imagesPath);
    void calibration();
    void saveParam();

    Size chessBoardSize;
    Size rectSize;
    Size imageSize;
    int patternType;
    string imgPath;
private:
    vector<vector<Point2f>> chessBoardCorners;
    vector<vector<Point3f>> objectCorners;
    Mat cameraMatrix=Mat(3,3,CV_32FC1,Scalar::all(0));
    Mat distCoeffs=Mat(1,5,CV_32FC1,Scalar::all(0));

    void get3dCorners(vector<Point3f> &objPoints);

};


#endif //DRONEVISION_CALIBRATION_H
