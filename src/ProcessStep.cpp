//
// Created by hero on 19-6-13.
//

#include "ProcessStep.h"


//find the locate of armor,return the pixel position of it's four corners
void armorDetection(cv::Mat &frame,vector<cv::Point[4]> &corners){

}
//input armor
void pnpSolution(vector<cv::Point[4]> corners,cv::Mat &positionInfo){

}

void ballisticSolution(cv::Mat positionInfo,int droneState[3],cv::Point &targetPoint){

}

Point decision(ArmorDetect &detector,AngleSolve &angleSolver) {
    if(detector.armors.empty()){
        angleSolver.targetPt=angleSolver.imgCenter;
        return angleSolver.targetPt;
    }
    vector<Point> targetPts;
    vector<Point3f> ptsPosition;
    for(int i=0;i<detector.armors.size();++i){
        Point temp=angleSolver.getAngle_Pixel(detector.armors[i].v_corners,detector.armors[i].isNormalArmor);
        Point3f tempDist=Point3f(angleSolver.CX,angleSolver.CY,angleSolver.CZ);
        targetPts.emplace_back(temp);
        ptsPosition.emplace_back(tempDist);
        circle(detector.sketch,temp,3,Scalar(255,255,255));
    }
    int index=0;
    double distance=sqrt(pow(targetPts[0].x-detector.imgCenter.x,2)
                         +pow(targetPts[0].y-detector.imgCenter.y,2));
    for(int i=1;i<targetPts.size();++i){
        double temp=sqrt(pow(targetPts[i].x-detector.imgCenter.x,2)
                         +pow(targetPts[i].y-detector.imgCenter.y,2));
        if(temp<distance){
            distance=temp;
            index=i;
        }
    }
    detector.finalArmor=detector.armors[index];
    angleSolver.targetPt=targetPts[index];
    cout<<"target point:"<<targetPts[index]<<endl;
    cout<<"position:"<<ptsPosition[index]<<endl;
    return angleSolver.targetPt;
}

void printLoopTime(Timer &timer){    cout<<"time: "<<timer.getLoopTime()<<" ms"<<endl; }

void averageTime(Timer &timer){    cout<<"average time:"<<timer.averageTime()<<"ms"<<endl;}

void printFPS(Timer &timer){    cout<<"FPS: "<<setprecision(4)<<1/timer.getLoopTime()*1000<<" fps"<<endl;  }

void averageFPS(Timer &timer){    cout<<"average FPS:"<<1/timer.averageTime()*1000<<"fps"<<endl;}

void printDate(Timer &timer){  cout<<"Date: "<<timer.getDate()<<endl;  }

char droneWaitKey(Setting &setter){
    setter.key=waitKey(setter.droneWaitkey);
    switch(static_cast<char>(setter.key)){
        case 'x':
            setter.droneWaitkey=0;
            break;
        case 'c':
            setter.droneWaitkey=1;
            break;
        case 'v':
            setter.writerFlag=setter.writerFlag;
            setter.saveVideo=~setter.saveVideo;
            break;
        case 'p':
            setter.savePic=1;
            break;
        case 'q':
            exit(-2);
        default:
            setter.writerFlag=setter.saveVideo;
            break;
    }
    return setter.key;
}

void waitkeyAction(Setting &setter,ProcessVal &val){
    if(setter.savePic){
        vector<String> fileNames;
        char fileName[100];
        string savePath;
        glob(setter.picturePath,fileNames);
        sprintf(fileName,"%ld.jpg",fileNames.size());
        savePath=setter.picturePath+ static_cast<string>(fileName);
        imwrite(savePath,val.src);
        cout<<"save "<<savePath<<endl;
        setter.savePic=0;
    }
    if(setter.saveVideo){
        static uint64 frameCount=0;
        if(setter.writerFlag==0 && setter.saveVideo!=0){
            vector<String> fileNames;
            glob(setter.videoPath,fileNames);
            char outVideoName[100];
            sprintf(outVideoName, "../others/video/%ld.avi", fileNames.size()+1);
            cout<<"save video:"<<outVideoName<<endl;
            val.videoWriter.open(outVideoName, VideoWriter::fourcc('M', 'J', 'P', 'G'),
                                 100, Size(val.src.cols, val.src.rows), true);
            val.videoWriter<<val.src;
            frameCount++;
        }else{
            val.videoWriter<<val.src;
            frameCount++;
            cout<<"frame count:"<<frameCount<<endl;
        }
    }
}
union sint_char{
    char dataChar[2];
    short int dataInt;
};
void write2Serial(AngleSolve &solve, Serial &serial,bool isDetected) {
    char data[10];
    data[0]=0xff;
    sint_char offsetX;
    sint_char offsetY;
    short detaX,detaY;
    Point center=Point(solve.imgSize.width/2-solve.offset.x,solve.imgSize.height/2-solve.offset.y);
    if(isDetected){
        detaX=offsetX.dataInt=solve.targetPt.x;
        detaY=offsetY.dataInt=solve.targetPt.y;
    }else{
        detaX=offsetX.dataInt=center.x;
        detaY=offsetY.dataInt=center.y;
    }
    data[1]=offsetX.dataChar[0];
    data[2]=offsetX.dataChar[1];
    data[3]=offsetY.dataChar[0];
    data[4]=offsetY.dataChar[1];
    if(isDetected)
        data[5]=1;
    else
        data[5]=0;
    double err=sqrt(pow(detaX-center.x,2)+pow(detaY-center.y,2));
    cout<<"err:"<<err<<endl;
    if(err<15)
        data[6]=1;
    else
        data[6]=0;
    data[7]=0;
    data[8]=char((detaX+detaY)/10);
    data[9]=char(254);
    cout<<endl;
    size_t i=serial.writeBuffer(data,10);
    if(i==10){
        if(serial.showTxMsg){
            cout<<serial.portName<<" Tx:";
            for(int k=0;i<10;++i)
                cout<<" "<<int(data[k]);
            cout<<endl;
        }
    } else
        perror("serial send message failed.");
}

