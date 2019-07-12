//
// Created by hero on 19-5-31.
//

#include <ModeAchieve.h>

#include "ModeAchieve.h"

void* ModeAchieve::droneAttack(){

}

void* ModeAchieve::debugMode(){
    cout<<"****************debug mode***************"<<endl;

    Timer timer;
    cv::Mat frame;
    ProcessVal val;
    vector<Armor> armors;
    Point target;
    Camera camera("../others/paramList/cameraParam.yaml");
    Setting setter("../others/paramList/settingParam.yaml");
    AngleSolve angleSolver("../others/paramList/pnpParam.yaml");
    ArmorDetect detector("../others/paramList/armorDetectParam.yaml");

    camera.read(frame);
    detector.setSize(camera.imgSize,camera.offset);
    angleSolver.setSize(camera.imgSize,camera.offset);
    cout<<"start!!!"<<endl;
    do{
        camera.read(frame);
        if (frame.empty()) break;
        detector.findArmor(frame, armors);
        //angleSolver.getSerialData(RxMsg);
        target=decision(detector,angleSolver);
        if(armors.empty()) {
            cout << "skip..........." << endl;
            write2Serial(angleSolver, *serial, false);
        } else
            write2Serial(angleSolver,*serial,true);
        if(detector.showDraw){
            circle(detector.sketch,target,5,Scalar(0,255,0),2);
            imshow("sketch",detector.sketch);
        }
        droneWaitKey(setter);
        val.update(detector);
        waitkeyAction(setter,val);
        printFPS(timer);
    }while(setter.key!='q');
    averageTime(timer);
    averageFPS(timer);
}

void ModeAchieve::calibration(){
    cout<<"calibrating..."<<endl;
    Calibrate calibrator("../others/paramList/pnpParam.yaml");
    vector<String> imgNames;
    Mat frame;
    Camera camera("../others/paramList/cameraParam.yaml");
    int key=' ';
    /***********************获取图像*************************/
    cout<<"taking pictures..."<<endl;
    while(static_cast<char>(key)!='k'){
        camera.read(frame);
        imshow("frame",frame);
        key=waitKey(1);
        if(static_cast<char>(key)=='s'){
            string path="../others/calibration/image/";
            glob(path,imgNames);
            char imgName[100];
            sprintf(imgName,"../others/calibration/image/%ld.jpg",imgNames.size()+1);
            imwrite(imgName,frame);
            cout<<"save image to : "<< static_cast<string>(imgName)<<endl;
        }
    }
    /***********************开始标定*************************/
    cout<<"calibrating..."<<endl;
    calibrator.findChessBoardCorners("../others/calibration/image/");
    calibrator.calibration();
    calibrator.saveParam();
}

void ModeAchieve::trainNumberRecognizeModel(){

}

void* ModeAchieve::msgDeal() {
    if(serial->fd==-1){
        do{
            serial=new Serial("../others/paramList/serialParam.yaml");
            if(serial->fd!=-1)
                break;
        }while(serial->fd==-1);
    }
    do{
//        cout<<"read size:"<<serial->readBuffer(RxMsg,10)<<endl;
        if(serial->showRxMsg && serial->readBuffer(RxMsg,10)==10){
            cout<<serial->portName<<" check:";
            char c=RxMsg[1]+RxMsg[3]+RxMsg[5];
            if(c==RxMsg[9])
                cout<<"pass"<<endl;
            else
                cout<<"error"<<endl;
        }
    }while(STOP==0);
    return nullptr;
}

bool ModeAchieve::debugMode_2_thread() {
    thread t1(&ModeAchieve::debugMode,this);
    thread t2(&ModeAchieve::msgDeal,this);
    t1.join();
    t2.join();
    return false;
}

ModeAchieve::ModeAchieve() {
    serial =new Serial("../others/paramList/serialParam.yaml");
}
