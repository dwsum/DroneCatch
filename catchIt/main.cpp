
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc.hpp>
//#include "test2.avi"

using namespace std;
using namespace cv;

//VideoCapture cap(-1);
VideoCapture cap("/home/drew/Downloads/test2.avi");

void setUpCamera() {

    if(!cap.isOpened()) {
        cout << "Error opening video stream or file";
    }
}

void displayCamera() {

    while(1) {
        Mat frame;
        cap >> frame;

        if(frame.empty())
            break;

        imshow("Frame", frame);

        char c = (char)waitKey(25);
        if(c==27)
            break;
    }
}

void convertToGrayScale() {
    bool firstTime = true;
    Mat oldFrame;
    Mat newFrame;

    while(1) {
        Mat frame;
        Mat grayscale;
        Mat grayThreshold;
        Mat differenceFrame;

        cap >> frame;

        if(frame.empty()) {
            break;      //this would mean something went wrong
        }

        cv::cvtColor(frame, grayscale, CV_RGB2GRAY);        //convert to grayscale image
        cv::threshold(grayscale, grayThreshold, 100, 255, CV_THRESH_BINARY_INV);

        if(firstTime) {
            firstTime = false;
            oldFrame = grayThreshold;
        }
        newFrame = grayThreshold;

        cv::absdiff(oldFrame, newFrame, differenceFrame);

        imshow("Frame", frame);
        imshow("MyVideo", grayscale);
        imshow("grayThreshold", grayThreshold);
        imshow("difference One", differenceFrame);

        oldFrame = newFrame;        //set it up for the next round!

        char c = (char)waitKey(25);
        if(c==27)
            break;
    }
}

void closeCamera() {
    cap.release();

    destroyAllWindows();
}

int main() {

    setUpCamera();

    //displayCamera();

    convertToGrayScale();

    closeCamera();
    return 0;
}
