//
// Created by drew on 3/2/20.
//

#ifndef CATCHIT_CAMERA_H
#define CATCHIT_CAMERA_H

#define VIDEO_RESOLUTION_X 640
#define VIDEO_RESOLUTION_Y 480

class cameraControl {
public:
    cameraControl();        //sets up everything when defined.
    bool isDone();      //returns if the camera portion is done or not
    double getInitialXVelocity();
    double getInitialYVelocity();
    double getInitialZVelocity();

private:
    bool trackingStatus;        //false means it is done tracking
    Vec2i videoResolution;
    VideoCapture theSourceVideo;        //change it in the cameraControl() function if want something besides the camera.
};

cameraControl cameraControl::cameraControl() {
    trackingStatus = false;     //then when it checks it knows it isn't done.

    videoResolution = {VIDEO_RESOLUTION_X, VIDEO_RESOLUTION_Y};       //any more intense will make the pie too slow.

    theSourceVideo = VideoCapture(0, CAP_V4L2);
    theSourceVideo.set(CAP_PROP_FRAME_WIDTH, VIDEO_RESOLUTION_X);
    theSourceVideo.set(CAP_PROP_FRAME_WIDTH, VIDEO_RESOLUTION_Y);
    theSourceVideo.set(CV_CAP_PROP_FPS, 15);

    if(!theSourceVideo.isOpened()) {
        std::cout << "Ok. Something went wrong. The camera didn't open right." << std::endl;
    }
}

bool cameraControl::isDone() {
    return trackingStatus;
}


#endif //CATCHIT_CAMERA_H
