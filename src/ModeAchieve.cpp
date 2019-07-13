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
    ProcessVal val;
    vector<Armor> armors;
    vector<AreaX> areaXs;
    Point target;
    Setting setter("../others/paramList/settingParam.yaml");
    AngleSolve angleSolver("../others/paramList/pnpParam.yaml");
    ArmorDetect detector("../others/paramList/armorDetectParam.yaml");

    detector.setSize(camera->imgSize,camera->offset);
    angleSolver.setSize(camera->imgSize,camera->offset);
    cout<<"start!!!"<<endl;
    while(frame.empty()){}
    do{
        if (frame.empty()) continue;
        if(setter.detectMode==0){
            detector.findArmor(frame,armors);
            angleSolver.getSerialData(RxMsg);
            target=decision(detector,angleSolver,SOLVE_OBJ_ARMOR);
        }
        else{
            detector.findLEDX(frame,areaXs);
//            angleSolver.getSerialData(RxMsg);
            target=decision(detector,angleSolver,SOLVE_OBJ_AREAX);
        }

        if(serial->fd!=-1){
            armors.empty()?
            write2Serial(angleSolver,*serial,false):
            write2Serial(angleSolver,*serial,true);
        }
        if(detector.showDraw){
            circle(detector.sketch,target,5,Scalar(0,255,0),2);
//            resize(detector.sketch,detector.sketch,Size(640,400));
            imshow("sketch",detector.sketch);
        }
        droneWaitKey(setter);
        waitKeyFlag=setter.droneWaitkey;
        readOneFrame=setter.readOneFrame;
        val.update(detector);
        waitkeyAction(setter,val);
        printFPS(timer);
    }while(setter.key!='q');
    destroyAllWindows();
    averageTime(timer);
    averageFPS(timer);
    STOP=1;
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
        int i=10;
        do{
            serial=new Serial("../others/paramList/serialParam.yaml");
            if(serial->fd!=-1)
                break;
            else{
                delete serial;
                sleep(1);
            }
        }while(i--!=0);
        if(serial->fd==-1)
        return nullptr;
    }
    do{
        size_t c=serial->readBuffer(RxMsg,10);
        if(serial->showRxMsg && c==10){
            if(serial->showRxMsg){
                cout<<serial->portName<<"RxMsg:";
                for(int k=0;k<10;++k)
                    cout<<" "<<int(RxMsg[k]);
                cout<<endl;
            }
            cout<<serial->portName<<" check:";
            char cb=char((RxMsg[1]+RxMsg[3]+RxMsg[5])%255);
            if(cb==RxMsg[9])
                cout<<"pass"<<endl;
            else
                cout<<"error"<<endl;
        }
    }while(STOP==0);
    return nullptr;
}

bool* ModeAchieve::readImage() {
    do{
        Mat temp;
        do{
            if(waitKeyFlag || readOneFrame){
                readOneFrame=0;
                temp=Mat();
                camera->read(temp);
            }
        }while(temp.empty());
            frame=temp;
    }while(STOP==0);
    return nullptr;
}

bool ModeAchieve::debugMode_2_thread() {
    thread t1(&ModeAchieve::debugMode,this);
    thread t2(&ModeAchieve::msgDeal,this);
    thread t3(&ModeAchieve::readImage,this);
    t1.join();
    t2.join();
    t3.join();
    return true;
}

ModeAchieve::ModeAchieve() {
    serial =new Serial("../others/paramList/serialParam.yaml");
    camera =new Camera("../others/paramList/cameraParam.yaml");
}

