//
// Created by hero on 19-5-31.
//

#include "Calibrate.h"

Calibrate::Calibrate(string calibrationParam) {
    cout<<"Open Calibration Param List:"<<calibrationParam;
    FileStorage fs(calibrationParam,FileStorage::READ);
    fs.isOpened()?cout<<"    opened."<<endl<<endl:cout<<"    open failed."<<endl<<endl;
    if(!fs.isOpened())
        exit(-1);
    fs["cameraMatrix"]>>cameraMatrix;
    fs["distCoeffs"]>>distCoeffs;
    fs["chessBoardSize"]>>chessBoardSize;
    fs["rectSize"]>>rectSize;
    fs["patternType"]>>patternType;
    fs["imgPath"]>>imgPath;
    fs.release();
}

void Calibrate::findChessBoardCorners(string imagesPath) {
    vector<String> imgNames;
    glob(imagesPath,imgNames);
    if(imgNames.empty())  exit(-1);
    imageSize=imread(imgNames[0]).size();
    Mat frame,gray;
    vector<Point3f> objPoints;
    vector<Point2f> corners;
    get3dCorners(objPoints);
    for(int i=0;i<imgNames.size();++i) {
        frame = imread(imgNames[i],0);
        bool  isDetected=findChessboardCorners(frame,chessBoardSize,corners);
        cout<<"isDetected:"<<isDetected<<endl;
        if(!isDetected){
            cout<<"can't found corners in "<<imgNames[i]<<endl;
            continue;
        }
        else{
            if(corners.size()==chessBoardSize.area()){
                cornerSubPix(frame,corners,Size(5,5),Size(-1,-1),
                             TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 50, 0.001));
                if(corners.size()==chessBoardSize.area()){
                    chessBoardCorners.emplace_back(corners);
                    objectCorners.emplace_back(objPoints);
                }
            }
        }
        drawChessboardCorners(frame,chessBoardSize,corners,true);
        imshow("corners",frame);
        waitKey(200);
    }
    destroyAllWindows();
}

void Calibrate::calibration() {
    Mat rvec,tvec;
    calibrateCamera(objectCorners,chessBoardCorners,chessBoardSize,cameraMatrix,distCoeffs,rvec,tvec,0);
    cout<<"cameraMatrix:"<<endl<<cameraMatrix<<endl;
    cout<<"distCoeffs:"<<endl<<distCoeffs<<endl;
    time_t timer = time(nullptr);
    string fileName=format("../others/calibration/%s.yaml", ctime(&timer));
    FileStorage fs(fileName,FileStorage::WRITE);
    fs<<"cameraMatrix"<<cameraMatrix;
    fs<<"distCoeffs"<<distCoeffs;
    fs.release();
}

void Calibrate::saveParam() {

}

void Calibrate::get3dCorners(vector<Point3f> &objPoints) {
    for(int i=0;i<chessBoardSize.height;++i)
        for(int j=0;j<chessBoardSize.width;++j)
            objPoints.emplace_back(Point3f(i*rectSize.width,j*rectSize.height,0.0f));
}

