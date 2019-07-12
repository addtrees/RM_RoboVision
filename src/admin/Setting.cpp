//
// Created by hero on 19-6-17.
//

#include "admin/Setting.h"

Setting::Setting(string settingParamPath) {
    cout<<"Setting Param Path:"<<settingParamPath;
    FileStorage fs(settingParamPath,FileStorage::READ);
    fs.isOpened()?cout<<"    opened."<<endl<<endl:cout<<"    open failed."<<endl<<endl;
    if(!fs.isOpened())
        exit(-1);
    fs["showOrigin"]>>showOrigin;
    fs["showGray"]>>showGray;
    fs["showBinary"]>>showBinary;
    fs["showLEDbar"]>>showLEDbar;
    fs["showDraw"]>>showDraw;
    fs["printTxMsg"]>>printTxMsg;
    fs["printRxMsg"]>>printRxMsg;
    fs["printArmorPos"]>>printArmorPos;
    fs["printLoopTime"]>>printLoopTime;
    fs["droneWaitkey"]>>droneWaitkey;
    fs["saveVideo"]>>saveVideo;
    fs["savePic"]>>savePic;
    fs.release();
}
