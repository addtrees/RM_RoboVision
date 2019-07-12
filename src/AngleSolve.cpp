//
// Created by hero on 19-6-24.
//
#include "AngleSolve.h"

AngleSolve::AngleSolve(string pnpParamPath) {
    cout<<"Angle Solve Param Path:"<<pnpParamPath;
    FileStorage fs(pnpParamPath,FileStorage::READ);
    fs.isOpened()?cout<<" opened."<<endl<<endl:cout<<" open failed."<<endl<<endl;
    if(!fs.isOpened())
        exit(-1);
    fs["cameraMatrix"]>>cameraMatrix;
    fs["distCoffs"]   >>distCoeffs;
    fs.release();
    get3dPoint();
    focalX=cameraMatrix.at<double>(0,0);
    focalY=cameraMatrix.at<double>(1,1);
    cout<<"focalX:"<<focalX<<endl
        <<"focalY:"<<focalY<<endl;
}

void AngleSolve::setSize(Size size,Point offset) {
    this->imgSize=size;
    this->offset =offset;
    this->imgCenter =Point(imgSize.width/2-offset.x,imgSize.height/2-offset.y);
    cout<<"offset:"<<offset<<endl
        <<"image cennter:"<<imgCenter<<endl;
}

/******************************************************************
 * @param isNormalArmor
 * //        P1           P2
//
//
//              o********       Point3f(x,y,z=0)//cm
//               *       x
//               *
//        P0     *y    P3
 */
void AngleSolve::get3dPoint() {
    float x,y,xL,yL;
    x=normalArmorSize.width/2,y=normalArmorSize.height/2;
    xL=largeArmorSize.width/2,yL=largeArmorSize.height/2;
    vector<Point3f>{Point3f(-x,y,0),Point3f(-x,-y,0),Point3f(x,-y,0),Point3f(x,y,0)}.swap(normalArmorPts3D);
    vector<Point3f>{Point3f(-xL,yL,0),Point3f(-xL,-yL,0),Point3f(xL,-yL,0),Point3f(xL,yL,0)}.swap(largeArmorPts3D);
}

/*************************************************************
 * 参数中角度均为弧度制
 * @param distance 相机与物体的水平距离
 * @param height 相机与物体竖直方向上的高度差
 * @param min
 * @param max
 * @param targetAngle 弹道解算之后枪口需要对准的位置的绝对角度，高于水平面为正
 * @param accuracy
 * @param g
 * @param v
 */
bool computeAngle(Point3d coordinate,
                  double height,
                  double min,
                  double max,
                  double &targetAngle,
                  const double accuracy,
                  const double g,
                  const double v){
    double distance=sqrt(pow(coordinate.x,2)+pow(coordinate.y,2)+pow(coordinate.z,2));
    double theta=asin(height/distance);
    double tempMin=min,
           tempMax=max,
           middle =(min+max)/2;
    double lowY,upperY;
    height=-height;
    do{
        lowY  =tan(tempMin)*cos(theta)*distance-0.5*g*pow(distance,2)*pow(cos(theta),2)
                /(v*v*pow(cos(tempMin),2));
        upperY=tan(middle)*cos(theta)*distance-0.5*g*pow(distance,2)*pow(cos(theta),2)
                /(v*v*pow(cos(middle),2));
        lowY<height && upperY>height ?
            tempMax=middle,middle=(tempMin+tempMax)/2 :
            tempMin=middle,middle=(tempMax+tempMin)/2;
    }while(tempMax-tempMin > accuracy);
    targetAngle=middle;
}

/**********************************************************
 * Roll轴变换
 * @param targetPoint 计算之后的打击点
 * @param armorCenter 装甲中心
 * @param roll （弧度制）
 * @return
 */
Point rollTransform(Point &targetPoint,Point2f armorCenter,double roll){

    double raiseAngle=sqrt(pow(targetPoint.x-armorCenter.x,2)+pow(targetPoint.y-armorCenter.y,2));
    targetPoint.x=int(armorCenter.x-raiseAngle*sin(roll));
    targetPoint.y=int(armorCenter.y-raiseAngle*cos(roll));
    return targetPoint;
}

bool AngleSolve::getSerialData(char *data) {
    u_data altitudeRx;
    u_data rollRx;
    u_data bulletSpeedRx;
    altitudeRx.c_data[0]   =data[1];
    altitudeRx.c_data[1]   =data[2];
    rollRx.c_data[0]       =data[3];
    rollRx.c_data[1]       =data[4];
    bulletSpeedRx.c_data[0]=data[5];
    bulletSpeedRx.c_data[1]=data[6];
    altitude   =double(float(altitudeRx.s_data)/100);
    ptzRoll    =rollRx.s_data*CV_PI/180;
    bulletSpeed=bulletSpeedRx.s_data;
}

void AngleSolve::pnp(vector<Point2f> corners,
                     vector<Point3f> pts3D,
                     Point imgCenter,
                     Mat cameraMatrix,
                     Mat distCoeffs,
                     Mat &rvec,
                     Mat &tvec) {
    Point2f center=Point2f(imgCenter);
    vector<Point2f>{corners[0]-center,corners[1]-center,corners[2]-center,corners[3]-center}.swap(corners);
    solvePnP(pts3D,corners,cameraMatrix,distCoeffs,rvec,tvec, false,SOLVEPNP_ITERATIVE);
}

void AngleSolve::getTransformMatrix() {
    transformMatrix = Mat::eye(3, 3, CV_64F);
    transformMatrix.at<double>(0, 0) = cos(ptzYaw)*cos(ptzPitch);
    transformMatrix.at<double>(0, 1) = cos(ptzYaw)*sin(ptzRoll)*sin(ptzPitch) - cos(ptzRoll)*sin(ptzYaw);
    transformMatrix.at<double>(0, 2) = sin(ptzYaw)*sin(ptzRoll) + cos(ptzYaw)*cos(ptzRoll)*sin(ptzPitch);
    transformMatrix.at<double>(1, 0) = cos(ptzPitch)*sin(ptzYaw);
    transformMatrix.at<double>(1, 1) = cos(ptzYaw)*cos(ptzRoll) + sin(ptzYaw)*sin(ptzRoll)*sin(ptzPitch);
    transformMatrix.at<double>(1, 2) = cos(ptzRoll)*sin(ptzYaw)*sin(ptzPitch) - cos(ptzYaw)*sin(ptzRoll);
    transformMatrix.at<double>(2, 0) = - sin(ptzPitch);
    transformMatrix.at<double>(2, 1) = cos(ptzPitch)*sin(ptzRoll);
    transformMatrix.at<double>(2, 2) = cos(ptzRoll)*cos(ptzPitch);
}

void AngleSolve::getCCS() {
    CX=tvec.at<double>(0,0)/1000;
    CY=tvec.at<double>(0,1)/1000;
    CZ=tvec.at<double>(0,2)/1000;
}

void AngleSolve::getWCS() {
    WCS=transformMatrix*tvec+T;
    WX=WCS.at<double>(0,0)/1000;
    WY=WCS.at<double>(0,1)/1000;
    WZ=WCS.at<double>(0,2)/1000;
}

void AngleSolve::reconstruct2Img() {
    invert(transformMatrix,invTransform);//求逆矩阵
    Mat reCCS=Mat(3,1,CV_64FC1);         //逆运算得到的相机坐标
    Mat reWCS=Mat(3,1,CV_64FC1);         //由计算弹道的角度alpha和garma得到的初始化坐标系上的坐标
    double reCX,reCY,reCZ;
    double reWX,reWY,reWZ;
    reWX=100.0*cos(garma)*sin(alpha);
    reWY=100.0*sin(garma);
    reWZ=100.0*cos(garma)*cos(alpha);
    reWCS.at<double>(0,0)=reWX;
    reWCS.at<double>(0,1)=reWY;
    reWCS.at<double>(0,2)=reWZ;
    reCCS=invTransform*reWCS;
    reCX=reCCS.at<double>(0,0);
    reCY=reCCS.at<double>(0,1);
    reCZ=reCCS.at<double>(0,2);
    //cout<<"reCCS:"<<reCCS<<endl;
    ballisticPt.x=reCX/reCZ*focalX/100+imgSize.width/2;
    ballisticPt.y=reCY/reCZ*focalY/100+imgSize.height/2;
    cout<<"ballistcPt:"<<ballisticPt<<endl;
}

void AngleSolve::getAngle_3DSolve(vector<Point2f> &points, bool isNormalArmor) {
    vector<Point3f> armorCorners3D=isNormalArmor?normalArmorPts3D:largeArmorPts3D;
    Point center=Point(imgSize.width/2-30+offset.x,imgSize.height/2-35+offset.y);
    pnp(points,armorCorners3D,center,cameraMatrix,distCoeffs,rvec,tvec);
    getTransformMatrix();
    getCCS();
    getWCS();
    reconstructionPt.x=double(imgSize.width)/2+CX/CZ*focalX;
    reconstructionPt.y=double(imgSize.height)/2+CY/CZ*focalY;
    reconstruct2Img();
}

Point AngleSolve::getAngle_Pixel(vector<Point2f> &points, bool isNormalArmor) {
    //这里检查是为了防止传入错误的点，而不是应对没有检测到装甲的情况，没有检测到装甲就没有可传入的角点
    if(points.size()!=4){
        perror("the number of points for angle solve is not allow !!!");
        targetPt=imgCenter;
        return targetPt;
    }
    reconstructionPt.x=double(imgSize.width)/2+offset.x+CX/CZ*focalX;
    reconstructionPt.y=double(imgSize.height)/2+offset.y+CY/CZ*focalY;
    double height=altitude-baseHeight;
    vector<Point3f> armorCorners3D=isNormalArmor?normalArmorPts3D:largeArmorPts3D;
    Point center=Point(imgSize.width/2-30+offset.x,imgSize.height/2-35+offset.y);
    pnp(points,armorCorners3D,center,cameraMatrix,distCoeffs,rvec,tvec);
    getCCS();
    double lineDist=sqrt(CX*CX+CY*CY+CZ*CZ);  //相机与装甲的直线距离
    double theta=asin(height/lineDist);       //装甲和发射连线与竖直轴的夹角（弧度制）,恒为正
    double targetAngle;                       //弹道解算之后枪口需要对准的位置的绝对角度，高于水平面为正
    Point3d coordinate=Point3d(CX,CY,CZ);
    computeAngle(coordinate,height,-50*CV_PI/180,50*CV_PI/180,targetAngle,0.0001,g,bulletSpeed);
    if(showTargetAngle)
        cout<<"target angle:"<<targetAngle*180/CV_PI<<endl;
    /********角度解算关键部分*************/
    Point2f armorCenter;
    armorCenter=Point(points[0]+points[1]+points[2]+points[3])/4;
    double raiseAngle=theta+targetAngle;     //枪口轴线与相机和装甲连线之间的角度（弧度）
    double camrea2Axis=atan((armorCenter.y-imgCenter.y)/focalY);
    int detaY=int(focalY*(tan(raiseAngle-camrea2Axis)+tan(camrea2Axis)));
    targetPt.y=int(armorCenter.y-detaY);
    targetPt.x=int(armorCenter.x);
    rollTransform(targetPt,armorCenter,ptzRoll);
    if(showTargetPt)
        cout<<"targetPT:"<<targetPt<<endl;
    return targetPt;
}



