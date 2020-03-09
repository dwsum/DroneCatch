
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
//    Mat theContours;
    std::vector<Vec3f> theContours;
//    vector<vector<Point2i>> theContours;

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
//        cv::HoughCircles(differenceFrame, theContours, cv::HOUGH_GRADIENT_ALT, 1, 50000);
//        cv::findContours(differenceFrame, theContours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
//
//        std::cout << theContours.size() << " that is the size" << std::endl;
//


//        HoughCircles(differenceFrame, theContours, HOUGH_GRADIENT, 1, differenceFrame.rows/64, 200, 10, 5, 30);
//
//        for(size_t i = 0; i < theContours.size(); i++) {
//            Point center(cvRound(theContours[i][0]), cvRound(theContours[i][1]));
//            int radius = cvRound(theContours[i][2]);
//            circle(differenceFrame, center, radius, Scalar(255, 255, 255), 2, 8 , 0);
//        }

        HoughCircles(differenceFrame, theContours, HOUGH_GRADIENT, 1, differenceFrame.rows/64, 200, 10, 5, 30);

        for(size_t i = 0; i < theContours.size(); i++) {
            Point center(cvRound(theContours[i][0]), cvRound(theContours[i][1]));
            int radius = cvRound(theContours[i][2]);
            circle(differenceFrame, center, radius, Scalar(255, 255, 255), 2, 8 , 0);
        }

        imshow("Frame", frame);
        imshow("MyVideo", grayscale);
        imshow("grayThreshold", grayThreshold);
        imshow("difference One", differenceFrame);

        oldFrame = newFrame;        //set it up for the next round!

        char c = (char)waitKey(25);
        if(c==27)
            break;
    }

    std::cout << theContours.size() << std::endl;


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
