//
// Created by drew on 3/2/20.
//

#ifndef CATCHIT_CAMERA_H
#define CATCHIT_CAMERA_H

class cameraControl {
public:
    cameraControl();
    bool isDone();      //returns if the camera portion is done or not
private:
    bool trackingStatus;        //false means it is done tracking
};

cameraControl cameraControl::cameraControl() {
    trackingStatus = false;
}

bool cameraControl::isDone() {
    return trackingStatus;
}


#endif //CATCHIT_CAMERA_H
