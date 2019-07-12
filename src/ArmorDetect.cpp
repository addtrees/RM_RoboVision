//
// Created by hero on 19-5-31.
//
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

template <typename T>
void textToImage(Mat &src,T text,Point position){
    char str[100];
    stringstream ss;
    ss<<text;
    ss>>str;
    putText(src,str,position,FONT_HERSHEY_SIMPLEX,0.6,Scalar(0,0,255));
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
        fs["LEDHWRatioMin"]   >>LEDHWRatio.min;
        fs["LEDHWRatioMax"]   >>LEDHWRatio.max;
        fs["normalArmorSize"] >>normalArmorSize;
        fs["largeArmorSize"]  >>largeArmorSize;
        fs["barLMaxRatio"]    >>barLMaxRatio;
        fs["barWMaxRatio"]    >>barWMaxRatio;
        fs["barAMaxRatio"]    >>barAMaxRatio;
        fs["angleDiff"]       >>angleDiff;
        fs["centerDist"]      >>centerDist;
    }
    fs.release();
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
    if(showBinary){
        imshow("binary",binary);
        createTrackbar("Thresh","Slider",&thresh,255);
    }
    threshold(gray,binary,thresh,255,THRESH_BINARY);
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
                char str[20];
                sprintf(str,"H_W:%f",ellipse.size.height/ellipse.size.width);
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
                char str[20];
                sprintf(str,"Ar:%f",areaProportion);
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
        int ptRed=0,ptBlue=0;                                      //红蓝颜色点的数量
        for(int j=0;j<contours[i].size();++j){                     //统计红蓝颜色像素数量
            int pixelDiff=src.at<Vec3b>(contours[i][j])[0]-src.at<Vec3b>(contours[i][j])[2];
            int pixelSum =src.at<Vec3b>(contours[i][j])[0]+src.at<Vec3b>(contours[i][j])[2];
            //为蓝,且像素差值大于0 2两通道像素值之和的一定比例
            if(pixelDiff>0 && pixelDiff>pixelSum*pixelDiffRatio)
                ptBlue++;
            else if(pixelDiff<0 && -pixelDiff>pixelSum*pixelDiffRatio)
                ptRed++;
        }
        if(showDraw){
            char str[20];
            sprintf(str,"B:%d R:%d",ptBlue,ptRed);
            putText(sketch,str,ellipses[i].center,FONT_HERSHEY_SIMPLEX,0.4,Scalar(0,255,0));
        }
        if((enemyColor==BLUE && (float(ptBlue)/(ptBlue+ptRed))>pixelCount) ||  //符合设定颜色条件
            (enemyColor==RED && (float(ptRed) /(ptBlue+ptRed))>pixelCount)){
            if(showDraw){
                Point2f pts[4];
                ellipse(sketch,ellipses[i],Scalar(255,0,0),1,LINE_AA);
                ellipses[i].points(pts);
                for(int k=0;k<4;++k){
                    char str[4];
                    sprintf(str,"%d",k);
                    putText(sketch,str,pts[k],FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,255,0));
                }
            }
        }
        else{
            if(showDraw){
                char str[20];
                if(enemyColor==BLUE)
                    sprintf(str,"color:%f",float(ptBlue)/(ptBlue+ptRed));
                else
                    sprintf(str,"color:%f",float(ptRed)/(ptBlue+ptRed));
                putText(sketch,str,contours[i][0],FONT_HERSHEY_SIMPLEX,0.4,Scalar(255,255,255));
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
        for(int j=i+1;j<ellipses.size();++j){
            /***************计算中心距********************/
            float dist= static_cast<float>(sqrt(pow(ellipses[i].center.x-ellipses[j].center.x,2)+
                                                pow(ellipses[i].center.y-ellipses[j].center.y,2)));
            float averH=(ellipses[i].size.height+ellipses[j].size.height)/2;
            if(dist/averH > centerDist)  continue;    //最远按照大装甲的尺寸来算，中心距不超过灯条高度的4.04倍

            float angleErr=abs(ellipses[i].angle-ellipses[j].angle);
            if(angleErr > 90)  angleErr=180-angleErr;
            if(angleErr > angleDiff) continue;
            float heightRatio;
            heightRatio=ellipses[i].size.height>ellipses[j].size.height?
                        ellipses[i].size.height/ellipses[j].size.height:
                        ellipses[j].size.height/ellipses[i].size.height;
            if(heightRatio>barLMaxRatio) continue;   //两装甲灯条的长度之比不应过大
            Armor armor;
            armor.set(ellipses[i],ellipses[j]);
            if(dist>averH*2.5)                       //小装甲宽度与灯条高度之比2.3377,取2.5
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
            if(ratio>4) continue;
            armors.emplace_back(armor);
            if(showDraw)
                for(int k=0;k<4;++k){
                    line(sketch,armor.corners[k],armor.corners[(k+1)%4],Scalar(255,255,255),3);
                    textToImage(sketch,k,armor.corners[(k+1)%4]);
                }
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
//            char str[4];
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

/**************************************************************
 * 重置所有过程变量
 */
void ArmorDetect::resetAll() {
    vector<Mat>()          .swap(rectArmors);
    vector<Armor>()        .swap(armors);
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
