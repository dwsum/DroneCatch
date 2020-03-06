
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

VideoCapture cap(0);

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

    while(1) {
        Mat frame;
        Mat grayscale;
        Mat grayThreshold;

        cap >> frame;

        if(frame.empty()) {
            break;      //this would mean something went wrong
        }

        cv::cvtColor(frame, grayscale, CV_RGB2GRAY);        //convert to grayscale image
        cv::threshold(grayscale, grayThreshold, 200, 255, CV_THRESH_BINARY_INV);

        imshow("Frame", frame);
        imshow("MyVideo", grayscale);
        imshow("grayThreshold", grayThreshold);

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

//    setUpCamera();

    //displayCamera();

    convertToGrayScale();

    closeCamera();
    return 0;
}
