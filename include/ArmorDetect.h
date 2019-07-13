//
// Created by hero on 19-5-31.
//

#ifndef DRONEVISION_ARMORDETECT_H
#define DRONEVISION_ARMORDETECT_H

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

#define TOSTRING(input,output)
#define BLUE 0
#define RED 1
#define ALL 2
/*************************************************
 *    1*****2
 *    *     *
 *    *     *
 *    0*****3
 */
struct Armor{
public:
    void set(Point corner[4]);
    void set(RotatedRect &rect1,RotatedRect &rect2);
    Point center;
    Point2f corners[4];
    vector<Point2f> v_corners;
    bool empty= true;
    bool isNormalArmor= true;
};

struct AreaX{
public:
    void set(RotatedRect &rect1,RotatedRect &rect2);
    Point center;
    Point2f corners[4];
    vector<Point2f> v_corners;
    double axisAngle;
    bool empty=true;
};

struct Interval{ float min=0,max=0;};

class ArmorDetect {
public:
    explicit ArmorDetect(string detectParamPath);
    int findArmor(Mat &frame,vector<Armor> &armors);
    int findLEDX(Mat &frame,vector<AreaX> &areaX);
    void writeParams(string filePath);
    void setSize(Size size,Point offset);
    void resetAll();
    int enemyColor;
    Mat src;
    Mat gray;
    Mat binary;
    Mat sketch;
    Mat BGR2HSV_M=Mat(3,3,CV_64FC1);
    vector<Armor> armors;
    vector<Mat>   rectArmors;
    Armor         finalArmor;
    vector<AreaX> areaXs;
    AreaX         finalAreaX;
    Point offset   =Point(0,150);
    Size  imgSize  =Size(1280,800);
    Point imgCenter=Point(640,250);

/**************调试时控制显示信息的变量*********************/
    int showOrigin=0;
    int showGray=0;
    int showBinary=0;
    int showLEDbar=0;
    int showDraw=0;
    int showSlideBar=0;
private:
/************处理过程函数，检测过程分为几步********************/
    void toGray(Mat &src, Mat &gray,int show);
    void binaryProcess(Mat &gray,Mat &binary,int thresh,int show);
    void contoursProcess(Mat &binary,vector<vector<Point>> &contours);
    void LEDMatch(vector<vector<Point>> &contours,vector<Armor> &armors);
    void LEDMatchX(vector<vector<Point>> &contours,vector<AreaX> &areaXs);
    void transfromArmorPic(vector<Armor> &armors,vector<Mat> &rectArmor);

/************过程变量，每次检测之前将会由resetAll()清空********/
    vector<vector<Point>> contours;  //所有轮廓
    vector<vector<Point>> finalHulls;
    vector<vector<Point>> LEDBar;    //灯条轮廓
    vector<RotatedRect>   ellipses;

/***********从yaml参数表里读取进来的数据，方便随时调整,一般值不变*/
    Size2f normalArmorSize=Size(-1,-1);
    Size2f largeArmorSize =Size(-1,-1);
    Interval LEDHWRatio;       //单个灯条的宽高比
    int thresh          =-1;   //灰度图像阈值
    int minLEDBarLength =-1;   //灯条最小轮廓长度
    float hullRatio     =-1;   //轮廓面积与其凸包面积之比
    float pixelDiffRatio=-1;   //单个像素点像素值的差异
    float pixelCount    =-1;   //整个轮廓上红蓝像素点的统计结果，数量占比
    float purpleRatio   =-1;   //thread of C0/C2 ratio
    float barLMaxRatio  =-1;   //两灯条长度与平均值的比例
    float barWMaxRatio  =-1;   //两灯条宽度与平均值的比例
    float barAMaxRatio  =-1;   //两灯条面积与平均值的比例
    float angleDiff     =-1;   //两灯条的角度差异
    float centerDist    =-1;   //中心距与灯条长度均值的比值
    float barLMaxRatioX =-1;
    float centerDistX   =-1;   //八字形区域的灯条中心距
};
#endif //DRONEVISION_ARMORDETECT_H
