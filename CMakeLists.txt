cmake_minimum_required(VERSION 3.5.1)
project(DroneVision)
message("project name:" ${PROJECT_NAME})

find_package(OpenCV 3.4.3 REQUIRED)
if(OpenCV_FOUND)
    message("OpenCV Version:" ${OpenCV_VERSION})
else()
    message("OpenCV not found")
endif()

link_libraries("/usr/local/lib/libserial.so")
link_libraries("/lib/libMVSDK.so")
link_libraries("/usr/lib/x86_64-linux-gnu/libpthread.so")
#link_libraries("/usr/lib/i386-linux-gnu/libpthread.a")
#link_libraries("/home/hero/pyEnvironment/Torch/lib/python3.5/site-packages/torch/lib/libtorch.so")

include_directories("include")
include_directories("/usr/local/include")
include_directories("/usr/local/include/serial")
include_directories("include/IndustrialCamera")
include_directories("include/admin")
#include_directories("/home/hero/pyEnvironment/Torch/lib/python3.5/site-packages/torch/lib/include/torch/csrc/api/include/torch")
set(CMAKE_CXX_STANDARD 14)

add_executable(
        DroneVision
        src/main.cpp
        src/Serial.cpp include/Serial.h
        src/SpatialSolution.cpp   include/SpatialSolution.h
        src/ArmorDetect.cpp       include/ArmorDetect.h
        src/ModeAchieve.cpp       include/ModeAchieve.h
        src/Calibrate.cpp         include/Calibrate.h
        src/ProcessStep.cpp       include/ProcessStep.h
        src/Camera.cpp            include/Camera.h
        src/admin/ProcessVal.cpp  include/admin/ProcessVal.h
        src/admin/Setting.cpp     include/admin/Setting.h
        src/admin/Timer.cpp       include/admin/Timer.h
        src/MindVisionCamera.cpp  include/IndustrialCamera/MindVisionCamera.h
                                  include/IndustrialCamera/CameraDefine.h
                                  include/IndustrialCamera/CameraStatus.h
                                  include/IndustrialCamera/CameraApi.h
        src/AngleSolve.cpp include/AngleSolve.h)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})