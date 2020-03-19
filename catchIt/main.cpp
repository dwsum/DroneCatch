
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc.hpp>
#include "CalculatePosition.h"
//#include "test2.avi"


//these next few are for testing only. Remove later to save space/speed on pi
#include <chrono>

#define CHOSEN_WINDOW 60 //350 for pi, 60 for computer          //note in milliseconds. NOTE: This is milliseconds per frame
#define FRAMES_PER_SECOND 60//(1 / CHOSEN_WINDOW * 1000)        //note, this is the frames per second used in some caluclations

using namespace std;
using namespace cv;

VideoCapture cap(0);
//VideoCapture cap("/home/drew/Downloads/v2-air.mp4");
//VideoCapture cap("/home/drew/Downloads/march17.h264");


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

//this function is called when sorting the Contours...we only want the one that is the ball. and it will be the biggest.
bool compareContourAreas(const std::vector<Point2i>& contourOne, const vector<Point2i>& contourTwo) {
    double i = contourArea(contourOne, false);      //note, false is because we don't care about orientation
    double j = contourArea(contourTwo, false);

    return (i < j);
}

void findContours() {
    int cntrBall = 0;
    std::vector<cv::Point_<int>> firstPoint;
    std::vector<cv::Point_<int>> secondPoint;
    Point2f centerOne;
    float radiusOne;
    Point2f centerTwo;
    float radiusTwo;

    while(1) {
        auto start = chrono::high_resolution_clock::now();
        Mat image;
        cap >> image;
        bool allDone = false;

        if(image.empty()) {
            std::cout << "breaks" << std::endl;
            break;
        }

//        namedWindow( "Display window", CV_WINDOW_AUTOSIZE );
//        imshow( "Display window", image );

        Mat gray;
        cvtColor(image, gray, CV_BGR2GRAY);
        Canny(gray, gray, 100, 200, 3);

        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        RNG rng(12345);
        findContours( gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

        if(contours.size() > 1) {
//            /// Draw contours
//            Mat drawing = Mat::zeros( gray.size(), CV_8UC3 );
//
//            sort(contours.begin(), contours.end(), compareContourAreas);

            for(int i = 0; i < contours.size(); i++) {
                std::vector<cv::Point_<int>> ballContour = contours[i];

                Point2f foundCenter;
                float foundRadius;
                minEnclosingCircle(ballContour, foundCenter, foundRadius);
                if(foundRadius > 5 && foundRadius < 20) {
//                    std::cout << "found a contour " << contours.size() << std::endl;
//                    std::cout << "radius of " << foundRadius << std::endl;
//                for( int i = 0; i< contours.size(); i++ )
//                {
//                    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
//                    drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
//
//                }
//
//                imshow( "Result window", drawing );
                    if(cntrBall < 5) {
                        cntrBall++;
                        break;
                    }
                    else if(cntrBall == 5) {
                        cntrBall++;
                        centerOne = foundCenter;
                        radiusOne = foundRadius;
                        break;
                    }
                    else if(cntrBall == 6) {
                        cntrBall++;
                        centerTwo = foundCenter;
                        radiusTwo = foundRadius;
                        break;
                    }
                    else if(cntrBall > 6) {
                        allDone = true;
                        break;
                    }
                }
            }



        }

        if(allDone) {
            std::cout << "breaks the right way" << std::endl;
            break;
        }

        char c = (char)waitKey(25);
        if(c==27)
            break;

        auto end = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);
        std::cout << "the duration " << duration.count() << std::endl;
        if(duration.count() > CHOSEN_WINDOW) {
            std::cout << "the chosen window is too small. The duration here is " << duration.count() << std::endl;
        }
        while (duration.count() < CHOSEN_WINDOW) {
            end = chrono::high_resolution_clock::now();
            duration = chrono::duration_cast<chrono::milliseconds>(end - start);
        }
        std::cout << "the duration after is " << duration.count() << std::endl;
    }

    std::cout << "POint one x and y are (" << centerOne.x << ", " << centerOne.y << ")." << std::endl;
    std::cout << "POint two x and y are (" << centerTwo.x << ", " << centerTwo.y << ")." << std::endl;
    std::cout << "radius of the two " << radiusOne << " " << radiusTwo << std::endl;

    double focalLength = 3.6;           //the pi camera is 3.6. the web came is 6-infinity. Lets try 6. online documentation says the units on this is milimeters
    double ballRealDiameter = 127;       //this is in milimeters

    double distanceToBallOne = (focalLength * ballRealDiameter / (2* radiusOne)) * 0.1;       //Meters?
    double distanceToBallTwo = (focalLength * ballRealDiameter / (2 * radiusTwo)) * 0.1;      //Meters?

    std::cout << "Distance to the Ball One: " << distanceToBallOne << std::endl;
    std::cout << "Distance to the Ball Two: " << distanceToBallTwo << std::endl;

    //calculate the z heights
    double distanceXaxisBallOne = (centerOne.x - 320) * focalLength / 1000;
    double distanceYaxisBallOne = (centerOne.y - 240) * focalLength / 1000;
    double distanceXYplaneBallOne = sqrt(pow(distanceXaxisBallOne, 2) + pow(distanceYaxisBallOne, 2));

    double distanceXaxisBallTwo = (centerTwo.x - 320) * focalLength / 1000;
    double distanceYaxisBallTwo = (centerTwo.y - 240) * focalLength / 1000;
    double distanceXYplaneBallTwo = sqrt(pow(distanceXaxisBallTwo, 2) + pow(distanceYaxisBallTwo, 2));

    double zHeightBallOne = sqrt(pow(distanceToBallOne, 2) + pow(distanceXYplaneBallOne, 2));
    double zHeightBallTwo = sqrt(pow(distanceToBallTwo, 2) + pow(distanceXYplaneBallTwo, 2));

    double xVelocity = (centerTwo.x - centerOne.x) * FRAMES_PER_SECOND * focalLength / 1000;
    double yVelocity = (centerTwo.y - centerOne.y) * FRAMES_PER_SECOND * focalLength / 1000;
    double zVelocity = -(zHeightBallTwo - zHeightBallOne) * FRAMES_PER_SECOND;

    std::cout << "the x velocity is " << xVelocity <<std::endl;
    std::cout << "The y velocity is " << yVelocity << std::endl;
    std::cout << "The Z velocity is " << zVelocity << std::endl;

    double catchAltitude = 2;

    CalculatePosition dronePosition(xVelocity, yVelocity, zVelocity, zHeightBallTwo, catchAltitude, distanceXaxisBallTwo, distanceYaxisBallTwo);      //replaced distanceToBallTwo with zHeightBallTwo
    double timeToFall = dronePosition.getTime(catchAltitude);
    double xFinal = dronePosition.getFinalX(timeToFall);
    double yFinal = dronePosition.getFinalY(timeToFall);

    std::cout << "The final landing position is at (" << xFinal << ", " << yFinal << ")" << std::endl;
    std::cout << "The time caluclated is " << timeToFall << std::endl;

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
//        auto start = chrono::high_resolution_clock::now();
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

        auto start = chrono::high_resolution_clock::now();
        cv::absdiff(oldFrame, newFrame, differenceFrame);
        auto end = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
        std::cout << "the duration of absdiff " << duration.count() << std::endl;

//        if(differenceFrame) {


        start = chrono::high_resolution_clock::now();
            HoughCircles(grayThreshold, theContours, HOUGH_GRADIENT, 1, grayThreshold.rows/64, 200, 10, 5, 15);
        end = chrono::high_resolution_clock::now();
        duration = chrono::duration_cast<chrono::microseconds>(end - start);
        std::cout << "the duration of houghCirlces " << duration.count() << std::endl;
//        }

        if(theContours.size() != 0) {
            cntr++;
            if(cntr == 5) {
                firstPoint = theContours;
            }
            else if(cntr == 6) {
                secondPoint = theContours;
            }
            else if(cntr > 6) {
                break;
            }
        }

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

        theContours.clear();

//        char c = (char)waitKey(25);
//        if(c==27)
//            break;

//        auto end = chrono::high_resolution_clock::now();
//        auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);
//        std::cout << "the duration " << duration.count() << std::endl;
//        if(duration.count() > CHOSEN_WINDOW) {
//            std::cout << "the chosen window is too small. The duration here is " << duration.count() << std::endl;
//        }
//        while (duration.count() < CHOSEN_WINDOW) {
//            end = chrono::high_resolution_clock::now();
//            duration = chrono::duration_cast<chrono::milliseconds>(end - start);
//        }
//        std::cout << "the duration after is " << duration.count() << std::endl;
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

    double distanceToBallOne = (focalLength * ballRealDiameter / (2* radiusOne)) * 0.1;       //Meters?
    double distanceToBallTwo = (focalLength * ballRealDiameter / (2 * radiusTwo)) * 0.1;      //Meters?

    std::cout << "Distance to the Ball One: " << distanceToBallOne << std::endl;
    std::cout << "Distance to the Ball Two: " << distanceToBallTwo << std::endl;

    //calculate the z heights
    double distanceXaxisBallOne = abs(320 - centerOne.x);
    double distanceYaxisBallOne = abs(240- centerOne.y);
    double distanceXYplaneBallOne = sqrt(pow(distanceXaxisBallOne, 2) + pow(distanceYaxisBallOne, 2)) * focalLength / 1000;

    double distanceXaxisBallTwo = abs(320 - centerTwo.x);
    double distanceYaxisBallTwo = abs(240- centerTwo.y);
    double distanceXYplaneBallTwo = sqrt(pow(distanceXaxisBallTwo, 2) + pow(distanceYaxisBallTwo, 2)) * focalLength / 1000;

    double zHeightBallOne = sqrt(pow(distanceToBallOne, 2) + pow(distanceXYplaneBallOne, 2));
    double zHeightBallTwo = sqrt(pow(distanceToBallTwo, 2) + pow(distanceXYplaneBallTwo, 2));

    double xVelocity = (centerTwo.x - centerOne.x) * 60 * focalLength / 1000;
    double yVelocity = (centerTwo.y - centerOne.y) * 60 * focalLength / 1000;
    double zVelocity = -(zHeightBallTwo - zHeightBallOne) * 60;

    std::cout << "the x velocity is " << xVelocity <<std::endl;
    std::cout << "The y velocity is " << yVelocity << std::endl;
    std::cout << "The Z velocity is " << zVelocity << std::endl;

    double catchAltitude = 2;

    //CalculatePosition dronePosition(xVelocity, yVelocity, zVelocity, zHeightBallTwo, catchAltitude);      //replaced distanceToBallTwo with zHeightBallTwo
//    double timeToFall = dronePosition.getTime(catchAltitude);
//    double xFinal = dronePosition.getFinalX(timeToFall);
//    double yFinal = dronePosition.getFinalY(timeToFall);

//    std::cout << "The final landing position is at (" << xFinal << ", " << yFinal << ")" << std::endl;
//    std::cout << "The time caluclated is " << timeToFall << std::endl;

}

void closeCamera() {
    cap.release();

    destroyAllWindows();
}

int main() {

    setUpCamera();

//    displayCamera();

    //convertToGrayScale();

    findContours();

    closeCamera();
    return 0;
}
