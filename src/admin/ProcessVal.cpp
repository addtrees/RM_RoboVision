//
// Created by hero on 19-6-18.
//

#include "admin/ProcessVal.h"

void ProcessVal::update(ArmorDetect &detector) {
    src=detector.src;
    gray=detector.gray;
    binary=detector.binary;
    sketch=detector.sketch;
}