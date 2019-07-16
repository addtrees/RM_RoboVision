//
// Created by hero on 19-5-31.
//
#include <ArmorDetect.h>

#include "ArmorDetect.h"

void Armor::set(cv::Point corner[4]) {
    vector<Point2f>().swap(v_corners);
    for(int i=0;i<4;++i)
        corners[i]=corner[i];
    v_corners.emplace_back(corners[0]);
    v_corners.emplace_back(corners[1]);
    v_corners.emplace_back(corners[2]);
    v_corners.emplace_back(corners[3]);
    center=(corners[0]+corners[1]+corners[2]+corners[3])/4;
    empty= false;
}

void Armor::set(RotatedRect &rect1,RotatedRect &rect2){
    vector<Point2f>().swap(v_corners);
    Point2f pts1[4],pts2[4];
    Point2f p0,p1,p2,p3;
    if(rect1.center.x<rect2.center.x){
        rect1.points(pts1);
        rect2.points(pts2);
    } else{
        rect1.points(pts2);
        rect2.points(pts1);
    }
    p0=rect1.angle>=90?(pts1[1]+pts1[2])/2:(pts1[0]+pts1[3])/2;
    p1=rect1.angle>=90?(pts1[0]+pts1[3])/2:(pts1[1]+pts1[2])/2;
    p2=rect2.angle>=90?(pts2[0]+pts2[3])/2:(pts2[1]+pts2[2])/2;
    p3=rect2.angle>=90?(pts2[1]+pts2[2])/2:(pts2[0]+pts2[3])/2;
    corners[0]=p0+(p0-p1)/2;
    corners[1]=p1+(p1-p0)/2;
    corners[2]=p2+(p2-p3)/2;
    corners[3]=p3+(p3-p2)/2;
    v_corners.emplace_back(corners[0]);
    v_corners.emplace_back(corners[1]);
    v_corners.emplace_back(corners[2]);
    v_corners.emplace_back(corners[3]);
    center=(corners[0]+corners[1]+corners[2]+corners[3])/4;
    empty=false;
}


void AreaX::set(RotatedRect &rect1, RotatedRect &rect2) {
    vector<Point2f>().swap(v_corners);
    Point2f pts1[4],pts2[4];
    Point2f p0,p1,p2,p3;
    if(rect1.center.x<rect2.center.x){
        rect1.points(pts1);
        rect2.points(pts2);
    } else{
        rect1.points(pts2);
        rect2.points(pts1);
    }
    p0=rect1.angle>=90?(pts1[1]+pts1[2])/2:(pts1[0]+pts1[3])/2;
    p1=rect1.angle>=90?(pts1[0]+pts1[3])/2:(pts1[1]+pts1[2])/2;
    p2=rect2.angle>=90?(pts2[0]+pts2[3])/2:(pts2[1]+pts2[2])/2;
    p3=rect2.angle>=90?(pts2[1]+pts2[2])/2:(pts2[0]+pts2[3])/2;
    v_corners.emplace_back(corners[0]=p0);
    v_corners.emplace_back(corners[1]=p1);
    v_corners.emplace_back(corners[2]=p2);
    v_corners.emplace_back(corners[3]=p3);
    center=(p0+p1+p2+p3)/4;
    Point2f axis=((p0+p3)/2)-((p1+p2)/2);
    axisAngle=atan(axis.x/axis.y)*180/CV_PI;
    empty=false;
}

template <typename T>
void textToImage(Mat &src,T text,Point position){
    char str[100];
    stringstream ss;
    ss<<text;
    ss>>str;
    putText(src,str,position,FONT_HERSHEY_SIMPLEX,0.6,Scalar(0,255,255));
}

/**************************************************************
 *
 * @param detectParamPath :装甲检测参数表地址
 */
ArmorDetect::ArmorDetect(const string detectParamPath) {
    cout<<"Armor Detection Param List:"<<detectParamPath;
    cv::FileStorage fs(detectParamPath,cv::FileStorage::READ);
    fs.isOpened()?cout<<" opened."<<endl:cout<<" open failed."<<endl;
    if(!fs.isOpened())
        exit(-1);
    else{
        fs["enemyColor"]      >>enemyColor;
        fs["showOrigin"]      >>showOrigin;
        fs["showGray"]        >>showGray;
        fs["showBinary"]      >>showBinary;
        fs["showLEDbar"]      >>showLEDbar;
        fs["showDraw"]        >>showDraw;
        fs["showSlideBar"]    >>showSlideBar;
        fs["thresh"]          >>thresh;
        fs["minLEDBarLength"] >>minLEDBarLength;
        fs["hullRatio"]       >>hullRatio;
        fs["pixelDiffRatio"]  >>pixelDiffRatio;
        fs["pixelCount"]      >>pixelCount;
        fs["purpleRatio"]     >>purpleRatio;
        fs["LEDHWRatioMin"]   >>LEDHWRatio.min;
        fs["LEDHWRatioMax"]   >>LEDHWRatio.max;
        fs["normalArmorSize"] >>normalArmorSize;
        fs["largeArmorSize"]  >>largeArmorSize;
        fs["barLMaxRatio"]    >>barLMaxRatio;
        fs["barWMaxRatio"]    >>barWMaxRatio;
        fs["barAMaxRatio"]    >>barAMaxRatio;
        fs["angleDiff"]       >>angleDiff;
        fs["centerDist"]      >>centerDist;
        fs["barLMaxRatioX"]   >>barLMaxRatioX;
        fs["centerDistX"]     >>centerDistX;
    }
    fs.release();
    BGR2HSV_M.at<double>(0,0)=0.412411;
    BGR2HSV_M.at<double>(1,0)=0.357585;
    BGR2HSV_M.at<double>(2,0)=0.180454;
    BGR2HSV_M.at<double>(1,0)=0.212649;
    BGR2HSV_M.at<double>(1,1)=0.715169;
    BGR2HSV_M.at<double>(1,2)=0.072182;
    BGR2HSV_M.at<double>(2,0)=0.019332;
    BGR2HSV_M.at<double>(2,1)=0.119195;
    BGR2HSV_M.at<double>(2,2)=0.950390;
}

/**************************************************************
 * @function :向参数表写入参数
 * @param detectParamPath
 */
void ArmorDetect::writeParams(string filePath) {
    cv::FileStorage fs(filePath,cv::FileStorage::APPEND);
    fs<<"thresh"         <<thresh;
    fs<<"minLEDBarLength"<<minLEDBarLength;
    fs<<"LEDHWRatioMin"  <<LEDHWRatio.min;
    fs<<"LEDHWRatioMax"  <<LEDHWRatio.max;
    fs<<"normalArmorSize"<<normalArmorSize;
    fs<<"largeArmorSize" <<largeArmorSize;
    fs<<"barLMaxRatio"   <<barLMaxRatio;
    fs<<"barWMaxRatio"   <<barWMaxRatio;
    fs<<"barAMaxRatio"   <<barAMaxRatio;
    fs<<"angleDiff"      <<angleDiff;
    fs<<"centerDist"     <<centerDist;
    fs.release();
}

void ArmorDetect::toGray(Mat &src, Mat &gray,const int showGray){
    std::vector<Mat> channels;
    cvtColor(src,gray,COLOR_BGR2GRAY);
    if(showGray)  imshow("gray",gray);
}

void ArmorDetect::binaryProcess(Mat &gray,Mat &binary,int thresh,int showBinary) {
    threshold(gray,binary,thresh,255,THRESH_BINARY);
    if(showBinary){
        imshow("binary",binary);
        createTrackbar("Thresh","Slider",&thresh,255);
    }
}

    /**************************************************************
     * 除最外层轮廓，内层轮廓都只属于最外层的子轮廓，没有单层树结构
     * 父子关系索引保存在hierarchy中，hierarchy[i][0] ~hierarchy[i][3]
       分别表示第i个轮廓的后一个轮廓、前一个轮廓、父轮廓、内嵌轮廓的索引编号
     * 保存下每个点
     */
void ArmorDetect::contoursProcess(Mat &binary,vector<vector<Point>> &contours) {
    vector<vector<Point>>().swap(contours);
    findContours(binary,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    for(int i=0;i<contours.size();++i){
    /****移除过小轮廓，以组成轮廓的点的个数判定****/
        if(contours[i].size()<minLEDBarLength){
            contours.erase(contours.begin()+(i--));
            continue;
        }
    /****由轮廓长宽比过滤部分轮廓****************/
        RotatedRect ellipse;
        ellipse=fitEllipse(contours[i]);
        if(ellipse.size.height/ellipse.size.width > LEDHWRatio.max ||
           ellipse.size.height/ellipse.size.width < LEDHWRatio.min){
            if(showDraw){
                char str[100];
                sprintf(str,"H_W:%2.2f",ellipse.size.height/ellipse.size.width);
                putText(sketch,str,ellipse.center,FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,255,255));
            }
            contours.erase(contours.begin()+(i--));
            continue;
        }
        else
            ellipses.emplace_back(ellipse);
    }
    /****通过凸包与原轮廓的面积比滤除畸形轮廓******/
    vector<vector<Point>> hulls(contours.size());        //凸包轮廓
    for(int i=0;i<contours.size();++i){
        convexHull(Mat(contours[i]),hulls[i]);
        float contourA= static_cast<float>(contourArea(contours[i]));
        float hullA= static_cast<float>(contourArea(hulls[i]));
        float areaProportion=(hullA-contourA)/hullA;    //面积占比
        if(areaProportion>hullRatio){
            if(showDraw){
                char str[100];
                sprintf(str,"Ar:%2.2f",areaProportion);
                putText(sketch,str,contours[i][0],FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,255,255));
            }
            contours.erase(contours.begin()+i);
            ellipses.erase(ellipses.begin()+i);
            hulls.erase(hulls.begin()+(i--));
            continue;
        }
    }

    /***************************************************************
     * 通过统计轮廓上每个点的颜色，判定该轮廓是否为所选轮廓
     * 将符合颜色条件的轮廓都压入hulls中，进行下一步匹配工作
     */
    for(int i=0;i<contours.size();++i){
        int ptRed=0,ptBlue=0,ptPurple=0;                           //红蓝颜色点的数量
        int channel0=0,channel1=0,channel2=0;
        for(int j=0;j<contours[i].size();++j){                     //统计红蓝颜色像素数量
            Mat BGR(3,1,CV_64FC1);
            Mat HSV(3,1,CV_64FC1);
            channel0=src.at<Vec3b>(contours[i][j])[0];
            channel1=src.at<Vec3b>(contours[i][j])[1];
            channel2=src.at<Vec3b>(contours[i][j])[2];
            BGR.at<double>(0,0)=channel0;
            BGR.at<double>(0,1)=channel1;
            BGR.at<double>(0,2)=channel2;
            HSV=BGR2HSV_M*BGR;
            int pixelDiff=channel0-channel2;
            int pixelSum =channel0+channel2;
            //为蓝,且像素差值大于0 2两通道像素值之和的一定比例
            if(pixelDiff>0 && pixelDiff>pixelSum*pixelDiffRatio)
                ptBlue++;
            else if(pixelDiff<0 && -pixelDiff>pixelSum*pixelDiffRatio)
                ptRed++;
            else if((float(channel0)/channel2>1.0/purpleRatio)
                  &&(float(channel0)/channel2<purpleRatio)
                  &&(channel0+channel2)/2>channel1){
                ptPurple++;
//                cout<<"BGR:"<<BGR<<" HSV:"<<HSV<<endl;
            }

        }
        if(showDraw){
            char str[100];
            sprintf(str,"B:%1d R:%1d P:%1d",ptBlue,ptRed,ptPurple);
            putText(sketch,str,ellipses[i].center,FONT_HERSHEY_SIMPLEX,0.3,Scalar(0,255,0));
        }
        //the contours which can fit this condition is suitable for the color select
        if((enemyColor==BLUE && (float(ptBlue)/contours[i].size())>pixelCount)||  //符合设定颜色条件
           (enemyColor==RED  && (float(ptRed)/contours[i].size())>pixelCount) ||
                                (float(ptPurple/contours[i].size()>pixelCount*0.9))){
            if(showDraw)
                enemyColor==RED?
                ellipse(sketch,ellipses[i],Scalar(255,0,0),2,LINE_AA):
                ellipse(sketch,ellipses[i],Scalar(0,0,255),2,LINE_AA);
        }
        else{
            if(showDraw){
                char str[100];
                if(enemyColor==BLUE){
                    sprintf(str,"BLUE:%2.2f",float(ptBlue)/contours[i].size());
                    putText(sketch,str,contours[i][0],FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,0,0));
                }
                else if(enemyColor==RED){
                    sprintf(str,"RED:%2.2f",float(ptRed)/contours[i].size());
                    putText(sketch,str,contours[i][0],FONT_HERSHEY_SIMPLEX,0.4,Scalar(0,0,255));
                }else if(ptPurple>ptBlue&&ptPurple>ptRed){
                    sprintf(str,"PUR:%2.2f",float(ptBlue)/contours[i].size());
                    putText(sketch,str,contours[i][0],FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,0,255));
                }
            }
            contours.erase(contours.begin()+i);
            ellipses.erase(ellipses.begin()+(i--));
        }
    }
}

/***************************************************************
* 对初步筛选之后的轮廓进行方位匹配，得到可能是装甲的灯条组合
*/
void ArmorDetect::LEDMatch(vector<vector<Point>> &contours,vector<Armor> &armors) {
    vector<Armor>().swap(armors);
    for(int i=0;i<ellipses.size();++i){
//        char str[10];                          //查看拟合椭圆的角度
//        sprintf(str,"%1.0f",ellipses[i].angle);
//        putText(src,str,ellipses[i].center,FONT_HERSHEY_SIMPLEX,0.4,Scalar(0,255,0));\
//        imshow("test",src);
        for(int j=i+1;j<ellipses.size();++j){
            //校核两灯条的角度差
            float angleErr=abs(ellipses[i].angle-ellipses[j].angle);
            if(angleErr > 90)  angleErr=180-angleErr;
            if(angleErr > angleDiff) continue;
            //校核两灯条长度差异
            float heightRatio;
            heightRatio=ellipses[i].size.height>ellipses[j].size.height?
                        ellipses[i].size.height/ellipses[j].size.height:
                        ellipses[j].size.height/ellipses[i].size.height;
            if(heightRatio>barLMaxRatio) continue;
            //计算中心距
            float dist= static_cast<float>(sqrt(pow(ellipses[i].center.x-ellipses[j].center.x,2)+
                                                pow(ellipses[i].center.y-ellipses[j].center.y,2)));
            float averH=(ellipses[i].size.height+ellipses[j].size.height)/2;
            if(dist/averH > centerDist)  continue;    //最远按照大装甲的尺寸来算，中心距不超过灯条高度的4.04倍
            Armor armor;
            armor.set(ellipses[i],ellipses[j]);
            if(dist>averH*2.34)                       //小装甲宽度与灯条高度之比2.3377
                armor.isNormalArmor=false;
            //检验装甲的两对角线之间夹角是否过小
            Point2f diagonal1=armor.corners[0]-armor.corners[2];
            Point2f diagonal2=armor.corners[1]-armor.corners[3];
            double angle=asin((diagonal1.x*diagonal2.y-diagonal1.y*diagonal2.x)
                    /(sqrt(pow(diagonal1.x,2)+pow(diagonal1.y,2))
                    *sqrt(pow(diagonal2.x,2)+pow(diagonal2.y,2))))*180/CV_PI;
//            putText(sketch,to_string(angle),armor.corners[0]+Point2f(10,10),FONT_HERSHEY_SIMPLEX,0.5,Scalar(255,255,0));
            if(angle<30)  continue;
            double l1=sqrt(pow(diagonal1.x,2)+pow(diagonal1.y,2));
            double l2=sqrt(pow(diagonal2.x,2)+pow(diagonal2.y,2));
            double ratio=l1>l2?l1/l2:l2/l1;
            if(ratio>5)  continue;
            armors.emplace_back(armor);
            if(showDraw)
                for(int k=0;k<4;++k){
                    line(sketch,armor.corners[k],armor.corners[(k+1)%4],Scalar(255,255,255),3);
                    textToImage(sketch,k,armor.corners[k]);
                }
        }
    }
}


void ArmorDetect::LEDMatchX(vector<vector<Point>> &contours, vector<AreaX> &areaXs) {
    vector<AreaX>().swap(areaXs);
    for(int i=0;i<contours.size();++i){
        for(int j=i+1;j<contours.size();++j){
            //校核角度是否符合实际几何关系
            float angleErr=abs(ellipses[i].angle-ellipses[j].angle);
            if((angleErr<54)||(angleErr>84)){
                char str[10];
                sprintf(str,"%1.0f",angleErr);
                textToImage(sketch,str,(ellipses[i].center+ellipses[j].center)/2);
                continue;
            }
            //校核两灯条长度差异
            float heightRatio;
            heightRatio=ellipses[i].size.height>ellipses[j].size.height?
                        ellipses[i].size.height/ellipses[j].size.height:
                        ellipses[j].size.height/ellipses[i].size.height;
            if(heightRatio>barLMaxRatioX)      continue;
            //校核中心距
            float dist= static_cast<float>(sqrt(pow(ellipses[i].center.x-ellipses[j].center.x,2)+
                                                pow(ellipses[i].center.y-ellipses[j].center.y,2)));
            float averH=(ellipses[i].size.height+ellipses[j].size.height)/2;
            if(dist/averH > centerDistX)       continue;
            AreaX areaX;
            areaX.set(ellipses[i],ellipses[j]);
            char str[10];
            sprintf(str,"%1.1f",areaX.axisAngle);
            textToImage(src,str,areaX.center);
            ellipse(src,ellipses[i],Scalar(255,0,0));
            ellipse(src,ellipses[j],Scalar(255,0,0));
//            imshow("test",src);
            if(abs(areaX.axisAngle)>10)        continue;
            for(int k=0;k<4;++k){
                line(sketch,areaX.corners[k],areaX.corners[(k+1)%4],Scalar(255,255,255),3);
            }
            areaXs.emplace_back(areaX);
        }
    }
}

/**********由轮廓长宽比过滤部分轮廓**************/
void ArmorDetect::transfromArmorPic(vector<Armor> &armors,vector<Mat> &rectArmor) {
    int H=28,W=28;
    vector<Mat>().swap(rectArmor);
    cv::Point2f dstPoint[4]{cv::Point(0,H),cv::Point(0,0),cv::Point(W,0),cv::Point(W,H)};
    for (auto &armor : armors) {
        cv::Mat rect;
        cv::Mat warp_mat=getPerspectiveTransform(armor.corners,dstPoint);
        cv::warpPerspective(src,rect,warp_mat,cv::Size(H,W));
        rectArmors.emplace_back(rect);
//        if(showDraw){
//            char str[10];
//            sprintf(str,"%d",i);
//            imshow(str,rectArmor);
//        }
    }
//
}

void drawCenterLine(Mat &sketch,Point center){
    line(sketch,Point(0,center.y),Point(sketch.size().width,center.y),Scalar(0,0,255));
    line(sketch,Point(center.x,0),Point(center.x,sketch.size().height),Scalar(0,0,255));
}
/**************************************************************
 *
 * @param src    ：输入参数
 * @param armors ：输出参数
 * @return       ：装甲数量
 */
int ArmorDetect::findArmor(cv::Mat &frame, vector<Armor> &armors) {
    if(frame.empty()) return -1;
    src=frame;
    if(showDraw)      frame.copyTo(sketch);
    if(showOrigin)    imshow("src",src);
    if(showSlideBar)  namedWindow("Slider",WINDOW_AUTOSIZE);
    resetAll();
    drawCenterLine(sketch,imgCenter);
    toGray(src,gray,showGray);
    binaryProcess(gray,binary,thresh,showBinary);
    contoursProcess(binary,contours);
    LEDMatch(contours,armors);
    transfromArmorPic(armors,rectArmors);
    this->armors=armors;
}

/****************************************************************
 *
 * @param frame :输入图像
 * @param areaX ：输出参数
 * @return      ：寻找到的对象数量
 */
int ArmorDetect::findLEDX(Mat &frame, vector<AreaX> &areaXs) {
    if(frame.empty()) return -1;
    src=frame;
    if(showDraw)      frame.copyTo(sketch);
    if(showOrigin)    imshow("src",src);
    if(showSlideBar)  namedWindow("Slider",WINDOW_AUTOSIZE);
    resetAll();
    drawCenterLine(sketch,imgCenter);
    toGray(src,gray,showGray);
    binaryProcess(gray,binary,thresh,showBinary);
    contoursProcess(binary,contours);
    LEDMatchX(contours,areaXs);
    this->areaXs=areaXs;
    return 0;
}
/**************************************************************
 * 重置所有过程变量
 */
void ArmorDetect::resetAll() {
    vector<Mat>()          .swap(rectArmors);
    vector<Armor>()        .swap(armors);
    vector<AreaX>()        .swap(areaXs);
    vector<RotatedRect>()  .swap(ellipses);
    vector<vector<Point>>().swap(LEDBar);
    vector<vector<Point>>().swap(contours);
    vector<vector<Point>>().swap(finalHulls);
}

void ArmorDetect::setSize(const Size size,const Point offset) {
    this->imgSize  =size;
    this->offset   =offset;
    this->imgCenter=Point(size.width/2-offset.x,size.height/2-offset.y);
}



