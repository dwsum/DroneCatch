
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

int main() {
    VideoCapture cap(0);

    if(!cap.isOpened()) {
        cout << "Error opening video stream or file";
        return -1;
    }

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
    cap.release();

    destroyAllWindows();

    return 0;
}
