//
// Created by hero on 19-6-24.
//

#ifndef DRONEVISION_PNPSOLUTION_H
#define DRONEVISION_PNPSOLUTION_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>

#define SOLVE_OBJ_ARMOR 111
#define SOLVE_OBJ_AREAX 112

using namespace std;
using namespace cv;

union u_data{
    short s_data=0;
    char  c_data[2];
};

class AngleSolve {
public:
    explicit AngleSolve(string pnpParamPath);
    //基于世界坐标重建的解算方法，解算出对应的P,Y轴角度
    void getAngle_3DSolve(vector<Point2f> &points, bool isNormalArmor);
    //基于相机坐标解算的方法，最终计算出打击点在图像上的像素位置
    Point getAngle_Pixel(vector<Point2f> &points,int solveObj,bool isNormalArmor=false);
    bool  getSerialData(char *data);
    void  setSize(Size size,Point offset);
    Point offset;
    Size  imgSize=Size(1280,800);
    Point imgCenter=Point(imgSize.width/2-offset.x,imgSize.height/2-offset.y);
    Point targetPt;                                 //解算后应该对着图像中心的点
    double CX,CY,CZ;                                //装甲板在相机坐标系的坐标
    double ptzYaw=0,ptzPitch=0,ptzRoll=0*CV_PI/180;
    Point2d reconstructionPt;                       //由PnP解算出来的装甲的位置信息在图像上的重投影点
    Point2d ballisticPt;                            //由位置信息解算弹道后反向重建于图像上的投影点

    /*******从串口传输过来的数据**********/
    double altitude=1.79;                           //无人机高度
    double ptzAngleV[3]={0.0,0.0,0.0};              //云台角速度
    double ptzAngleA[3]={0.0,0.0,0.0};              //云台角加速度

private:
    /********debug控制变量*************/
    int showTargetAngle=1;
    int showTargetPt=0;
    /*********处理过程变量**************/
    void get3dPoint();
    void pnp(vector<Point2f> corners,
             vector<Point3f> pts3D,
             Point imgCenter,
             Mat cameraMatrix,
             Mat distCoeffs,
             Mat &rvec,
             Mat &tvec);
    void getTransformMatrix();
    void calcHeightDist(Mat CCS,Mat WCS);
    void getCCS();
    void getWCS();
    void reconstruct2Img();
    Mat rvec,tvec;
    Mat transformMatrix=Mat(3,3,CV_64FC1);       //从世界坐标系到相机坐标系的变换矩阵
    Mat invTransform=Mat(3,3,CV_64FC1);          //变换矩阵逆矩阵
    Mat WCS=Mat(3,1,CV_64FC1);                   //世界坐标world coordinate
    Mat T=Mat(3,1,CV_64FC1,Scalar::all(0.0));    //相机坐标系原点相对于世界坐标系原点的坐标
    double WX,WY,WZ;                             //装甲经过坐标变换之后的世界坐标
    double alpha=0,garma=0;                      //绝对坐标系下枪管的旋转角度和抬起角度
    double targetPitch,targetYaw;                //最终目标变量
    vector<Point3f> largeArmorPts3D;
    vector<Point3f> normalArmorPts3D;
    vector<Point3f> areaXPts3D;

    /*******************从参数表读入********************/
    Mat cameraMatrix;
    Mat distCoeffs;
    double focalX,focalY;

    /*********************恒量*************************/
    const double baseHeight=1.52 ;                //基地距离地面高度
    const Size2f normalArmorSize=Size2f(130.4,111.56);
    const Size2f largeArmorSize =Size2f(225.4,111.56);
    //从结构祖给的solidworks模型中测量得到的
    float  areaXSizeRatio                =0.67;
    const float  areaXTop       =float(57.79*areaXSizeRatio);     //57.79
    const float  areaXDown      =float(150.9*areaXSizeRatio);     //150.9
    const float  areaXHeight    =float(38.77*areaXSizeRatio);     //38.77
    double bulletSpeed=28;
    const double g = 9.7949;                      //nanjing
    //const double g=9.7887;                        //shenzhen
    //const double g=9.7944;                        //xi'an

};




#endif //DRONEVISION_PNPSOLUTION_H
