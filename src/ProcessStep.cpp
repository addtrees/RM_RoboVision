//
// Created by hero on 19-6-13.
//

#include <ProcessStep.h>

#include "ProcessStep.h"


//find the locate of armor,return the pixel position of it's four corners
void armorDetection(cv::Mat &frame,vector<cv::Point[4]> &corners){

}
//input armor
void pnpSolution(vector<cv::Point[4]> corners,cv::Mat &positionInfo){

}

void ballisticSolution(cv::Mat positionInfo,int droneState[3],cv::Point &targetPoint){

}

Point decision(ArmorDetect &detector,AngleSolve &angleSolver,int solveObj) {
    if(!(solveObj==SOLVE_OBJ_ARMOR ||solveObj==SOLVE_OBJ_AREAX)){
        perror("solve type is unsurported!");
        angleSolver.targetPt=angleSolver.imgCenter;
        return angleSolver.imgCenter;
    }
    if((solveObj==SOLVE_OBJ_ARMOR&&detector.armors.empty())||
       (solveObj==SOLVE_OBJ_AREAX&&detector.areaXs.empty())){
        angleSolver.targetPt=angleSolver.imgCenter;
        return angleSolver.targetPt;
    }
    vector<Point> targetPts;
    vector<Point3f> ptsPosition;
    if(solveObj==SOLVE_OBJ_ARMOR){
        for(int i=0;i<detector.armors.size();++i){
            Point temp=angleSolver.getAngle_Pixel(detector.armors[i].v_corners,
                                                  SOLVE_OBJ_ARMOR,
                                                  detector.armors[i].isNormalArmor);
            Point3f tempDist=Point3f(angleSolver.CX,angleSolver.CY,angleSolver.CZ);
            targetPts.emplace_back(temp);
            ptsPosition.emplace_back(tempDist);
            circle(detector.sketch,temp,3,Scalar(255,255,255));
        }
    } else{
        for(int i=0;i<detector.areaXs.size();++i){
            Point temp =angleSolver.getAngle_Pixel(detector.areaXs[i].v_corners,SOLVE_OBJ_AREAX);
//            Point tempA=angleSolver.getAngle_Pixel(detector.armors[i].v_corners,SOLVE_OBJ_ARMOR,detector.armors[i].isNormalArmor);
            Point3f tempDist=Point3f(angleSolver.CX,angleSolver.CY,angleSolver.CZ);
            targetPts.emplace_back(temp);
            ptsPosition.emplace_back(tempDist);
            circle(detector.sketch,angleSolver.reconstructionPt,4,Scalar(0,0,255),2);
            circle(detector.sketch,temp,2,Scalar(255,255,255));
        }
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
    if(solveObj==SOLVE_OBJ_ARMOR)
        detector.finalArmor=detector.armors[index];
    else
        detector.finalAreaX=detector.areaXs[index];
    angleSolver.targetPt=targetPts[index];
//    cout<<"target point:"<<targetPts[index]<<endl;
//    cout<<"distance:"<<sqrt(pow(ptsPosition[index].x,2)+pow(ptsPosition[index].y,2)+pow(ptsPosition[index].z,2))<<endl;
    return angleSolver.targetPt;
}

Point decisionX(ArmorDetect &detector, AngleSolve &angleSolver) {
    if(detector.areaXs.empty()){
        angleSolver.targetPt=angleSolver.imgCenter;
        return angleSolver.targetPt;
    }
    vector<Point> targetPts;
    vector<Point3f> ptsPosition;

    return cv::Point();
}

void printLoopTime(Timer &timer){    cout<<"time: "<<timer.getLoopTime()<<" ms"<<endl; }

void averageTime(Timer &timer){    cout<<"average time:"<<timer.averageTime()<<"ms"<<endl;}

void printFPS(Timer &timer){    cout<<"FPS: "<<setprecision(4)<<1/timer.getLoopTime()*1000<<" fps"<<endl;  }

void averageFPS(Timer &timer){    cout<<"average FPS:"<<1/timer.averageTime()*1000<<"fps"<<endl;}

void printDate(Timer &timer){  cout<<"Date: "<<timer.getDate()<<endl;  }

char droneWaitKey(Setting &setter){
    setter.key=char(waitKey(setter.droneWaitkey));
    switch(setter.key){
        case 'x':
            if(setter.droneWaitkey==0)
                setter.readOneFrame=1;
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
            setter.key='q';
            break;
        default:
            if(setter.droneWaitkey==0)
                setter.readOneFrame=1;
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
                                 25, Size(val.src.cols, val.src.rows), true);
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
void write2Serial(AngleSolve &solve, Serial &serial,char *TxMsg,bool isDetected) {
    char data[10];
    data[0]=0xff;
    sint_char offsetX;
    sint_char offsetY;
    short detaX,detaY;
    int x=5;
    int y=3;
    Point center=Point(solve.imgSize.width/2-solve.offset.x+x,solve.imgSize.height/2-solve.offset.y+y);
    if(isDetected){
        detaX=offsetX.dataInt=solve.targetPt.x+x;
        detaY=offsetY.dataInt=solve.targetPt.y+y;
    }else{
        detaX=offsetX.dataInt=center.x;
        detaY=offsetY.dataInt=center.y;
    }
    data[1]=offsetX.dataChar[0];
    data[2]=offsetX.dataChar[1];
    data[3]=offsetY.dataChar[0];
    data[4]=offsetY.dataChar[1];
    if(isDetected){
        data[5]=1;
        double err=sqrt(pow(detaX-center.x,2)+pow(detaY-center.y,2));
//        printf("err:%2.2f\n",err);
        data[6]=err<15?char(1):char(0);
    }
    else{
        data[5]=0;
        data[6]=0;
//        printf("err:inf");
    }
    data[7]=0;
    data[8]=char((detaX+detaY)/10);
    data[9]=char(254);
    for(int i=0;i<10;++i)
        TxMsg[i]=data[i];
    size_t i=serial.writeBuffer(data,10);
    if(i==10){
        if(serial.showTxMsgInTime){
            cout<<serial.portName<<" Tx:";
            for(int k=0;k<10;++k)
                cout<<" "<<int(data[k]);
            cout<<endl;
        }
    } else
        perror("serial send message failed");
}

