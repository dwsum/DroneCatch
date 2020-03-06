////
//// Created by drew on 3/2/20.
////
//
//#ifndef CATCHIT_CAMERA_H
//#define CATCHIT_CAMERA_H
//
//#define VIDEO_RESOLUTION_X 640
//#define VIDEO_RESOLUTION_Y 480
//
//#include <opencv2/opencv.hpp>
//#include <opencv2/videoio.hpp>
//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//
//class cameraControl {
//public:
//    cameraControl();
//
//    //sets up everything when defined.
//    bool isDone()  {
//        return trackingStatus;
//    }  //returns if the camera portion is done or not
//    double getInitialXVelocity();
//    double getInitialYVelocity();
//    double getInitialZVelocity();
//
//private:
//    bool trackingStatus;        //false means it is done tracking
////    cv::Vec2i videoResolution;
//    cv::VideoCapture theSourceVideo;        //change it in the cameraControl() function if want something besides the camera.
//};
//
//
//
//cameraControl::cameraControl() {
//    trackingStatus = false;     //then when it checks it knows it isn't done.
//
////    videoResolution = {VIDEO_RESOLUTION_X, VIDEO_RESOLUTION_Y};       //any more intense will make the pie too slow.
//
////    theSourceVideo = cv::VideoCapture(0, cv::CAP_V4L2);
////    theSourceVideo.set(cv::CAP_PROP_FRAME_WIDTH, VIDEO_RESOLUTION_X);
////    theSourceVideo.set(cv::CAP_PROP_FRAME_WIDTH, VIDEO_RESOLUTION_Y);
//
////    if(!theSourceVideo.isOpened()) {
////        std::cout << "Ok. Something went wrong. The camera didn't open right." << std::endl;
////    }
//}
//
//
//
//
//#endif //CATCHIT_CAMERA_H
