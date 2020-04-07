
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc.hpp>
#include "CalculatePosition.h"
#include "math.h"
//#include "test2.avi"

//these next few are for testing only. Remove later to save space/speed on pi
#include <chrono>

#define CHOSEN_WINDOW 75 //350 for pi, 60 for computer          //note in milliseconds. NOTE: This is milliseconds per frame
#define FRAMES_PER_SECOND ((1.0 / CHOSEN_WINDOW) * 1000)        //note, this is the frames per second used in some caluclations

#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 480

#define PI 3.14159265
#define FOCAL_LENGTH 3.6 //module v2 focalLength is 3. 04. module v1 is 3.6.
#define HORIZONTAL_FIELD_OF_VIEW ((53.50 / 2) * (PI / 180))    //V1 is ((53.50 / 2) * (PI / 180)), v2 is ((62.2 / 2) * (PI / 180))
#define VERTICAL_FIELD_OF_VIEW ((41.41 / 2)  * (PI / 180))    //v1 is ((41.41 / 2)  * (PI / 180)), v2 is ((48.8 / 2)  * (PI / 180))

using namespace std;
using namespace cv;

//VideoCapture cap(0);
VideoCapture cap("/home/drew/Downloads/v2-air.mp4");
//VideoCapture cap("/home/drew/Downloads/march17.h264");


void setUpCamera() {

    if(!cap.isOpened()) {
        std::cout << "Error opening video stream or file" << std::endl;
    }
    cap.set(CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH);
    cap.set(CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT);
}

void calculateLand(Point2f centerOne, Point2f centerTwo, float radiusOne, float radiusTwo) {
    double maxAngle_xyPlane_zAxis = atan(sqrt(pow(tan(HORIZONTAL_FIELD_OF_VIEW), 2)+pow(tan(VERTICAL_FIELD_OF_VIEW), 2)));
    double focalLength = FOCAL_LENGTH;
    double ballRealDiameter = 127;       //this is in milimeters
    double rightAngle = PI / 2;             //change if we are doing radians...
    //following are going to be computed in here.
    //for Ball One
    double ballOne_fullDistance;    //this is the distance to directily to the ball
    double ballOne_thetaZ;          //angle between xy plane and the vector to the ball
    double ballOne_xyPixelValue;    //the number of pixels to the ball
    double ballOne_xyDistance;      //this is the distance (in meters) to the ball in xy plane.
    double ballOne_zHeight;         //this is the height of the ball in only z direction
    double ballOne_xDistance;       //this is the distance in the X direction to the ball only
    double ballOne_yDistance;       //this is the distance in the Y direction to the ball only
    double ballOne_xyTheta;         //this is the angle that tan(ballOnexyTheta) = ypixel/xpixel = yDistance/xDistance
    //for Ball Two
    double ballTwo_fullDistance;    //this is the distance to directily to the ball
    double ballTwo_thetaZ;          //angle between xy plane and the vector to the ball
    double ballTwo_xyPixelValue;    //the number of pixels to the ball
    double ballTwo_xyDistance;      //this is the distance (in meters) to the ball in xy plane.
    double ballTwo_zHeight;         //this is the height of the ball in only z direction
    double ballTwo_xDistance;       //this is the distance in the X direction to the ball only
    double ballTwo_yDistance;       //this is the distance in the Y direction to the ball only
    double ballTwo_xyTheta;         //this is the angle that tan(ballTwoxyTheta) = ypixel/xpixel = yDistance/xDistance
    //same for both balls
    double maxPixelAway;    //the number of pixels any of the corners are away from the center of pic

    //calculate the distance to the Balls.
    ballOne_fullDistance = (focalLength * ballRealDiameter / (2 * radiusOne)) * 0.1;       //meters
    ballTwo_fullDistance = (focalLength * ballRealDiameter / (2 * radiusTwo)) * 0.1;       //meters

    //calculate thetaZ (remember, its the theta between the vector to the ball and the xy plane)
    //in radians
    ballOne_xyPixelValue = sqrt(pow((centerOne.x - (VIDEO_WIDTH / 2)), 2) + pow((centerOne.y - (VIDEO_HEIGHT / 2)), 2));
    ballTwo_xyPixelValue = sqrt(pow((centerTwo.x - (VIDEO_WIDTH / 2)), 2) + pow((centerTwo.y - (VIDEO_HEIGHT / 2)), 2));
    maxPixelAway = sqrt(pow((VIDEO_HEIGHT / 2), 2) + pow((VIDEO_WIDTH / 2), 2));
    ballOne_thetaZ = rightAngle - (maxAngle_xyPlane_zAxis * (ballOne_xyPixelValue / maxPixelAway));
    ballTwo_thetaZ = rightAngle - (maxAngle_xyPlane_zAxis * (ballTwo_xyPixelValue / maxPixelAway));

    //calculate the thetaXY (remember, it is the angle that tan(ballTwoxyTheta) = ypixel/xpixel = yDistance/xDistance
    //in radians
    ballOne_xyTheta = atan(centerOne.y / centerOne.x);
    ballTwo_xyTheta = atan(centerTwo.y / centerTwo.x);

    //calculate the Distances now!! Boo-yah!!!!
    ballOne_xyDistance = ballOne_fullDistance * cos(ballOne_thetaZ);
    ballTwo_xyDistance = ballTwo_fullDistance * cos(ballTwo_thetaZ);

    ballOne_zHeight = ballOne_fullDistance * sin(ballOne_thetaZ);
    ballTwo_zHeight = ballTwo_fullDistance * sin(ballOne_thetaZ);


    //direction doesn't matter till here!!! Because above, is for calculating zHeight. Which is the same regardless of direction.
    ballOne_xDistance = ballOne_xyDistance * cos(ballOne_xyTheta);
    ballTwo_xDistance = ballTwo_xyDistance * cos(ballTwo_xyTheta);

    ballOne_yDistance = ballOne_xyDistance * sin(ballOne_xyTheta);
    ballTwo_yDistance = ballTwo_xyDistance * sin(ballTwo_xyTheta);

    //add direction in.
    //NOTE:
    //      NEGATIVE X is code for west
    //      NEGATIVE Y is code for North
    if(centerOne.x < (VIDEO_WIDTH / 2)) {
        ballOne_xDistance = -ballOne_xDistance;
    }
    if(centerOne.y < (VIDEO_HEIGHT / 2)) {
        ballOne_yDistance = -ballOne_yDistance;
    }
    if(centerTwo.x < (VIDEO_WIDTH / 2)) {
        ballOne_xDistance = -ballOne_xDistance;
    }
    if(centerTwo.y < (VIDEO_HEIGHT / 2)) {
        ballOne_yDistance = -ballOne_yDistance;
    }

    //same as end of predictLandLocation.
    std::cout << FRAMES_PER_SECOND << std::endl;
    double xVelocity = (ballTwo_xDistance - ballOne_xDistance) * FRAMES_PER_SECOND;
    double yVelocity = (ballTwo_yDistance - ballOne_yDistance) * FRAMES_PER_SECOND;
    double zVelocity = -(ballTwo_zHeight - ballOne_zHeight) * FRAMES_PER_SECOND;

    std::cout << "the x velocity is " << xVelocity << std::endl;
    std::cout << "The y velocity is " << yVelocity << std::endl;
    std::cout << "The Z velocity is " << zVelocity << std::endl;
    std::cout << "the x axis ball one " << ballOne_xDistance << std::endl;
    std::cout << "the x axis ball two " << ballTwo_xDistance << std::endl;
    std::cout << "the y axis ball one " << ballOne_yDistance << std::endl;
    std::cout << "the y axis ball two " << ballTwo_yDistance << std::endl;
    std::cout << "the z height ball one " << ballOne_zHeight << std::endl;
    std::cout << "the z height ball two " << ballTwo_zHeight << std::endl;
    std::cout << "focal length " << focalLength << std::endl;

    double catchAltitude = 0;

    CalculatePosition dronePosition(xVelocity, yVelocity, zVelocity, ballTwo_zHeight, catchAltitude,
                                    ballTwo_xDistance,
                                    ballTwo_yDistance);      //replaced distanceToBallTwo with zHeightBallTwo
    double timeToFall = dronePosition.getTime(catchAltitude);
    double xFinal = dronePosition.getFinalX(timeToFall);
    double yFinal = dronePosition.getFinalY(timeToFall);

    std::cout << "The final landing position is at (" << xFinal << ", " << yFinal << ")" << std::endl;
    std::cout << "The time caluclated is " << timeToFall << std::endl;


}

void predictLandLocation(Point2f centerOne, Point2f centerTwo, float radiusOne, float radiusTwo) {
    double focalLength = 3.6;            //module v2 focalLength is 3. 04. module v1 is 3.6. the web came is 6-infinity. Lets try 6. online documentation says the units on this is milimeters
    double ballRealDiameter = 127;       //this is in milimeters
    //following are going to be computed in here.
    double thetaZ;          //angle between xy plane and the vector to the ball
    double xyPixelValue;    //the number of pixels to the ball
    double maxPixelAway;    //the number of pixels any of the corners are away from the center of pic
    double xyDistance;      //this is the distance (in meters) to the ball in xy plane.
    double zHeight;

    std::cout << "POint one x and y are (" << centerOne.x << ", " << centerOne.y << ")." << std::endl;
    std::cout << "POint two x and y are (" << centerTwo.x << ", " << centerTwo.y << ")." << std::endl;
    std::cout << "radius of the two " << radiusOne << " " << radiusTwo << std::endl;



    double distanceToBallOne = (focalLength * ballRealDiameter / (2 * radiusOne)) * 100;       //millimeters.
    double distanceToBallTwo = (focalLength * ballRealDiameter / (2 * radiusTwo)) * 100;      //millimeters

    std::cout << "Distance to the Ball One: " << distanceToBallOne << std::endl;
    std::cout << "Distance to the Ball Two: " << distanceToBallTwo << std::endl;

/*
    new way to calculate z heights.
    plan: take the distance to Ball One/Two as the hypotenous, two sides
              triangle:
                  --Hypotenous: the distance to Ball One/Two. Found above. aka "Known"
                  --one side of triangle:the z height. Solving for. aka "unknown"
                  --other side of triangle: the combination of x and y in that direction aka "unknown"
                  --thetaOne: angle that touches the hypotonous and "other side of triangle" Known.
                          --this is slightly complicated. But, true all the same.
                          --To solve: do 90 - (90*(xyPixelValue/maxAway))
                                --xyPixelValue: the number of pixels away from the center of picture.
                                --maxAway: the number of pixels away any of the corners are from the picture

*/


    //calculate the z heights
    double distanceXaxisBallOne =
            (centerOne.x - (VIDEO_WIDTH / 2)) * focalLength / 1000.0;     //replaced 320 with 648...then with 1296
    double distanceYaxisBallOne =
            (centerOne.y - (VIDEO_HEIGHT / 2)) * focalLength / 1000.0;     //replaced 240 with 486...then with 972
    double distanceXYplaneBallOne = sqrt(pow(distanceXaxisBallOne, 2) + pow(distanceYaxisBallOne, 2));

    double distanceXaxisBallTwo =
            (centerTwo.x - (VIDEO_WIDTH / 2)) * focalLength / 1000.0;     //replaced 320 with 648...then with 1296
    double distanceYaxisBallTwo =
            (centerTwo.y - (VIDEO_HEIGHT / 2)) * focalLength / 1000.0;     //replaced 240 with 486..then with 972
    double distanceXYplaneBallTwo = sqrt(pow(distanceXaxisBallTwo, 2) + pow(distanceYaxisBallTwo, 2));

    double zHeightBallOne = sqrt(pow(distanceToBallOne, 2) + pow(distanceXYplaneBallOne, 2));
    double zHeightBallTwo = sqrt(pow(distanceToBallTwo, 2) + pow(distanceXYplaneBallTwo, 2));

//    double zHeightBallOne = sqrt(pow(distanceToBallOne, 2) + pow(distanceXaxisBallOne, 2));
//    double zHeightBallTwo = sqrt(pow(distanceToBallTwo, 2) + pow(distanceXaxisBallTwo, 2));


    std::cout << FRAMES_PER_SECOND << std::endl;
    double xVelocity = (centerTwo.x - centerOne.x) * FRAMES_PER_SECOND * focalLength / 1000;
    double yVelocity = (centerTwo.y - centerOne.y) * FRAMES_PER_SECOND * focalLength / 1000;
    double zVelocity = -(zHeightBallTwo - zHeightBallOne) * FRAMES_PER_SECOND;

    std::cout << "the x velocity is " << xVelocity << std::endl;
    std::cout << "The y velocity is " << yVelocity << std::endl;
    std::cout << "The Z velocity is " << zVelocity << std::endl;
    std::cout << "the x axis ball one " << distanceXaxisBallOne << std::endl;
    std::cout << "the x axis ball two " << distanceXaxisBallTwo << std::endl;
    std::cout << "the y axis ball one " << distanceYaxisBallOne << std::endl;
    std::cout << "the y axis ball two " << distanceYaxisBallTwo << std::endl;
    std::cout << "the z height ball one " << zHeightBallOne << std::endl;
    std::cout << "the z height ball two " << zHeightBallTwo << std::endl;
    std::cout << "focal length " << focalLength << std::endl;

    double catchAltitude = 0;

    CalculatePosition dronePosition(xVelocity, yVelocity, zVelocity, zHeightBallTwo, catchAltitude,
                                    distanceXaxisBallTwo,
                                    distanceYaxisBallTwo);      //replaced distanceToBallTwo with zHeightBallTwo
    double timeToFall = dronePosition.getTime(catchAltitude);
    double xFinal = dronePosition.getFinalX(timeToFall);
    double yFinal = dronePosition.getFinalY(timeToFall);

    std::cout << "The final landing position is at (" << xFinal << ", " << yFinal << ")" << std::endl;
    std::cout << "The time caluclated is " << timeToFall << std::endl;


}

//the next bunch of lines is what Lorena and I had together.
//    std::cout << "POint one x and y are (" << centerOne.x << ", " << centerOne.y << ")." << std::endl;
//    std::cout << "radius of the two " << radiusOne << " " << radiusTwo << std::endl;
//
//    double focalLength = 3.6;           //the pi camera is 3.6. the web came is 6-infinity. Lets try 6. online documentation says the units on this is milimeters
//    double ballRealDiameter = 127;       //this is in milimeters
//
//    double distanceToBallOne = (focalLength * ballRealDiameter / (2* radiusOne)) * 0.1;       //Meters?
//    double distanceToBallTwo = (focalLength * ballRealDiameter / (2 * radiusTwo)) * 0.1;      //Meters?
//
//    std::cout << "Distance to the Ball One: " << distanceToBallOne << std::endl;
//    std::cout << "Distance to the Ball Two: " << distanceToBallTwo << std::endl;
//
//    //calculate the z heights
//    double distanceXaxisBallOne = abs(320 - centerOne.x);
//    double distanceYaxisBallOne = abs(240- centerOne.y);
//    double distanceXYplaneBallOne = sqrt(pow(distanceXaxisBallOne, 2) + pow(distanceYaxisBallOne, 2)) * focalLength / 1000;
//
//    double distanceXaxisBallTwo = abs(320 - centerTwo.x);
//    double distanceYaxisBallTwo = abs(240- centerTwo.y);
//    double distanceXYplaneBallTwo = sqrt(pow(distanceXaxisBallTwo, 2) + pow(distanceYaxisBallTwo, 2)) * focalLength / 1000;
//
//    double zHeightBallOne = sqrt(pow(distanceToBallOne, 2) + pow(distanceXYplaneBallOne, 2));
//    double zHeightBallTwo = sqrt(pow(distanceToBallTwo, 2) + pow(distanceXYplaneBallTwo, 2));
//
//    double xVelocity = (centerTwo.x - centerOne.x) * 60 * focalLength / 1000;
//    double yVelocity = (centerTwo.y - centerOne.y) * 60 * focalLength / 1000;
//    double zVelocity = -(zHeightBallTwo - zHeightBallOne) * 60;
//
//    std::cout << "the x velocity is " << xVelocity <<std::endl;
//    std::cout << "The y velocity is " << yVelocity << std::endl;
//    std::cout << "The Z velocity is " << zVelocity << std::endl;
//
//    double catchAltitude = 2;

    //CalculatePosition dronePosition(xVelocity, yVelocity, zVelocity, zHeightBallTwo, catchAltitude);      //replaced distanceToBallTwo with zHeightBallTwo
//    double timeToFall = dronePosition.getTime(catchAltitude);
//    double xFinal = dronePosition.getFinalX(timeToFall);
//    double yFinal = dronePosition.getFinalY(timeToFall);

//    std::cout << "The final landing position is at (" << xFinal << ", " << yFinal << ")" << std::endl;
//    std::cout << "The time caluclated is " << timeToFall << std::endl;


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

                    if(cntrBall == 0) {
                        cntrBall++;
                        centerOne = foundCenter;
                        radiusOne = foundRadius;
                        break;
                    }
                    else if(cntrBall == 1) {
                        cntrBall++;
                        centerTwo = foundCenter;
                        radiusTwo = foundRadius;
                        allDone = true;
                        break;
                    }
                    else if(cntrBall > 1) {
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

    predictLandLocation(centerOne, centerTwo, radiusOne, radiusTwo);

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
        auto start = chrono::high_resolution_clock::now();
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

//        cv::absdiff(oldFrame, newFrame, differenceFrame);

        HoughCircles(grayThreshold, theContours, HOUGH_GRADIENT, 1, grayThreshold.rows/64, 200, 10, 5, 15);
//        end = chrono::high_resolution_clock::now();
//        duration = chrono::duration_cast<chrono::microseconds>(end - start);
//        std::cout << "the duration of houghCirlces " << duration.count() << std::endl;
//        }

        if(theContours.size() != 0) {
            if(cntr == 0) {
                firstPoint = theContours;
            }
            else if(cntr == 1) {
                secondPoint = theContours;
                break;
            }
            else if(cntr > 1) {
                break;
            }
            cntr++;
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

        oldFrame = newFrame;        //set it up for the next round!

        theContours.clear();

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

    //make the center and radius
    Point centerOne(cvRound(firstPoint[0][0]), cvRound(firstPoint[0][1]));
    int radiusOne = cvRound(firstPoint[0][2]);
    Point centerTwo(cvRound(secondPoint[0][0]), cvRound(secondPoint[0][1]));
    int radiusTwo = cvRound(secondPoint[0][2]);

    calculateLand(centerOne, centerTwo, radiusOne, radiusTwo);
}

void closeCamera() {
    cap.release();

    destroyAllWindows();
}

int main() {

    setUpCamera();

//    displayCamera();

    convertToGrayScale();

//    findContours();

    closeCamera();
    return 0;
}
