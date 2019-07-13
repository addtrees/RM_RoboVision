//
// Created by hero on 19-5-31.
//

#include <Serial.h>

#include "Serial.h"

Serial::Serial(string serialParamPath) {
    FileStorage fs(serialParamPath,FileStorage::READ);
    if(!fs.isOpened())  exit(-1);
    fs["portName"]>>portName;
    fs["showRxMsg"]>>showRxMsg;
    fs["showTxMsg"]>>showTxMsg;
    fs["inWar"]>>inWar;
    fs.release();

    cout<<"opening serial port: "<<portName<<" ";
    fd=open(portName.data(),O_RDWR | O_APPEND | O_NDELAY);
    if(fd==-1){
        cout<<"failed."<<endl;
        perror("\nserial open failed.\n");
        if(inWar)
            exit(-1);
    } else
        cout<<"  opened."<<endl;
//    fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);  //英雄上的串口设置
    fcntl(fd, F_SETFL, 0);                  //默认为阻塞模式
    struct termios port_settings;           //配置串口的结构体
    cfmakeraw(&port_settings);              //设置为原始模式,所有输入都按字节处理,实测没有这步配置,FF有时会读成7F甚至5F
    cfsetispeed(&port_settings, B115200);   //输入波特率
    cfsetospeed(&port_settings, B115200);   //输出波特率
//    port_settings.c_cflag &= ~PARENB;       //无奇偶校验 //c_cflag是16位的，相当于串口配置寄存器，按位配置
//    port_settings.c_cflag &= ~CSTOPB;       //无停止位
//    port_settings.c_cflag &= ~CSIZE;        //CSIZE占俩位，需先清零再配置不然配置为5、6、7时会失效
//    port_settings.c_cflag |= CS8;           //将字符大小设置为8位
//    port_settings.c_cflag |=(CLOCAL | CREAD);
    port_settings.c_cflag = 0x18b2;       //以上被注释语句等价于这步
    tcsetattr(fd, TCSANOW, &port_settings);//将配置串口结构体写入对应串口相应寄存器，TCSANOW表示立即生效
}

size_t Serial::readBuffer(char *inputBuffer, size_t bufferSize) {
    size_t size=read(fd,inputBuffer,bufferSize);
//    if(size==bufferSize){
//        if(showRxMsg){
//            cout<<"serial "<<portName<<" RxMsg:";
//            for(int i=0;i<bufferSize;++i)
//                cout<<int(inputBuffer[i])<<"\t";
//            cout<<endl;
//        } else
//            cout<<"read buffer failed."<<endl;
//        return bufferSize;
//    } else
        return size;
}

size_t Serial::writeBuffer(char *outputBuffer, size_t bufferSize) {
    if(bufferSize==write(fd,outputBuffer,bufferSize))
        return bufferSize;
    return 0;
}

bool Serial::readByte(char *inputData) {
    if(1==write(fd,inputData,1)){
        if(showRxMsg)
            cout<<"serial Rx byte: "<<inputData<<endl;
        return true;
    }
    return false;
}

bool Serial::writeByte(char *outputData) {
    if(1==write(fd,outputData,1)){
        if(showTxMsg)
            cout<<"serial Rx byte: "<<outputData<<endl;
        return true;
    }
    return false;
}

void Serial::closeSerial() {
    close(fd);
    cout<<"close_port:serial_port is closed"<<endl;
}
