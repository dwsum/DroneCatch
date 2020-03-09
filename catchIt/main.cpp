
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc.hpp>
//#include "test2.avi"

using namespace std;
using namespace cv;

//VideoCapture cap(-1);
VideoCapture cap("/home/drew/Downloads/v4-air.mp4");

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
    std::vector<Vec3f> theContours;
    int cntr = 0;
    std::vector<Vec3f> firstPoint;
    std::vector<Vec3f> secondPoint;

    while(1) {
        Mat frame;
        Mat grayscale;
        Mat grayThreshold;
//        Mat differenceFrame;

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

//        cv::absdiff(oldFrame, newFrame, differenceFrame);

        HoughCircles(grayThreshold, theContours, HOUGH_GRADIENT, 1, grayThreshold.rows/64, 200, 10, 5, 30);

        if(theContours.size() != 0) {
            cntr++;
            if(cntr == 4) {
                firstPoint = theContours;
            }
            else if(cntr == 5) {
                secondPoint = theContours;
            }
            else if(cntr > 5) {
                break;
            }
        }

        for(size_t i = 0; i < theContours.size(); i++) {
            Point center(cvRound(theContours[i][0]), cvRound(theContours[i][1]));
            int radius = cvRound(theContours[i][2]);
            circle(grayThreshold, center, radius, Scalar(255, 255, 255), 2, 8 , 0);
        }

        imshow("Frame", frame);
        imshow("MyVideo", grayscale);
        imshow("grayThreshold", grayThreshold);
//        imshow("difference One", differenceFrame);

        oldFrame = newFrame;        //set it up for the next round!

        theContours.clear();

        char c = (char)waitKey(25);
        if(c==27)
            break;
    }

    std::cout << theContours.size() << std::endl;

    std::cout << "at the end the cntr is at " << cntr << std::endl;

    Point centerOne(cvRound(firstPoint[0][0]), cvRound(firstPoint[0][1]));
    int radiusOne = cvRound(firstPoint[0][2]);

    Point centerTwo(cvRound(secondPoint[0][0]), cvRound(secondPoint[0][1]));
    int radiusTwo = cvRound(secondPoint[0][2]);

    std::cout << "POint one x and y are (" << centerOne.x << ", " << centerOne.y << ")" << std::endl;


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
