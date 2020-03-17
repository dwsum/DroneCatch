
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc.hpp>
#include "CalculatePosition.h"
//#include "test2.avi"


//these next few are for testing only. Remove later to save space/speed on pi
#include <chrono>


using namespace std;
using namespace cv;

//VideoCapture cap(0);
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

        auto start = chrono::high_resolution_clock::now();
        cv::cvtColor(frame, grayscale, CV_RGB2GRAY);        //convert to grayscale image
        auto stop = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
        std::cout << "this is duration of cvtColor: " << duration.count() << std::endl;

        start = chrono::high_resolution_clock::now();
        cv::threshold(grayscale, grayThreshold, 100, 255, CV_THRESH_BINARY_INV);
        stop = chrono::high_resolution_clock::now();
        duration = chrono::duration_cast<chrono::microseconds>(stop - start);
        std::cout << "this is duration of threshold: " << duration.count() << std::endl;

//        if(firstTime) {
//            firstTime = false;
//            oldFrame = grayThreshold;
//        }
//        newFrame = grayThreshold;

//        cv::absdiff(oldFrame, newFrame, differenceFrame);

        start = chrono::high_resolution_clock::now();
        HoughCircles(grayThreshold, theContours, HOUGH_GRADIENT, 1, grayThreshold.rows/64, 200, 10, 5, 30);
        stop = chrono::high_resolution_clock::now();
        duration = chrono::duration_cast<chrono::microseconds>(stop - start);
        std::cout << "this is duration of HoughCircles: " << duration.count() << std::endl;

        start = chrono::high_resolution_clock::now();
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
        stop = chrono::high_resolution_clock::now();
        duration = chrono::duration_cast<chrono::microseconds>(stop - start);
        std::cout << "this is duration of if statement: " << duration.count() << std::endl;


//        for(size_t i = 0; i < theContours.size(); i++) {
//            Point center(cvRound(theContours[i][0]), cvRound(theContours[i][1]));
//            int radius = cvRound(theContours[i][2]);
//            circle(grayThreshold, center, radius, Scalar(255, 255, 255), 2, 8 , 0);
//        }

//        imshow("Frame", frame);
//        imshow("MyVideo", grayscale);
//        imshow("grayThreshold", grayThreshold);
//        imshow("difference One", differenceFrame);

//        oldFrame = newFrame;        //set it up for the next round!

        start = chrono::high_resolution_clock::now();
        theContours.clear();
        stop = chrono::high_resolution_clock::now();
        duration = chrono::duration_cast<chrono::microseconds>(stop - start);
        std::cout << "this is duration of clearing: " << duration.count() << std::endl;
//        char c = (char)waitKey(25);
//        if(c==27)
//            break;
    }

    //make the center and radius
    Point centerOne(cvRound(firstPoint[0][0]), cvRound(firstPoint[0][1]));
    int radiusOne = cvRound(firstPoint[0][2]);
    Point centerTwo(cvRound(secondPoint[0][0]), cvRound(secondPoint[0][1]));
    int radiusTwo = cvRound(secondPoint[0][2]);

    std::cout << "POint one x and y are (" << centerOne.x << ", " << centerOne.y << ")." << std::endl;
    std::cout << "radius of the two " << radiusOne << " " << radiusTwo << std::endl;

    double focalLength = 3.6;           //the pi camera is 3.6. the web came is 6-infinity. Lets try 6. online documentation says the units on this is milimeters
    double ballRealDiameter = 127;       //this is in milimeters

    double distanceToBallOne = (focalLength * ballRealDiameter / (2* radiusOne));       //Meters?
    double distanceToBallTwo = (focalLength * ballRealDiameter / (2 * radiusTwo));      //Meters?

    std::cout << "Distance to the Ball One: " << distanceToBallOne << std::endl;
    std::cout << "Distance to the Ball Two: " << distanceToBallTwo << std::endl;

    double xVelocity = (centerTwo.x - centerOne.x) * 60 * focalLength / 1000;
    double yVelocity = (centerTwo.y - centerOne.y) * 60 * focalLength / 1000;
    double zVelocity = (distanceToBallTwo - distanceToBallOne) * 60;

    std::cout << "the x velocity is " << xVelocity <<std::endl;
    std::cout << "The y velocity is " << yVelocity << std::endl;
    std::cout << "The Z velocity is " << zVelocity << std::endl;

    double catchAltitude = 2;

    CalculatePosition dronePosition(xVelocity, yVelocity, zVelocity, distanceToBallTwo, catchAltitude);
    double timeToFall = dronePosition.getTime(catchAltitude);
    double xFinal = dronePosition.getFinalX(timeToFall);
    double yFinal = dronePosition.getFinalY(timeToFall);

    std::cout << "The final landing position is at (" << xFinal << ", " << yFinal << ")" << std::endl;
    std::cout << "The time caluclated is " << timeToFall << std::endl;

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
