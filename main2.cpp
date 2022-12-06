#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include "trackers/MyTracker.h"

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    if (argc != 2) {
        cerr << "Incorrect args" << endl;
        return 0;
    }
    VideoCapture capture(argv[1]);
    if (!capture.isOpened()) {
        cerr << "Unable to open file!" << endl;
        return 0;
    }
    namedWindow("Frame", WINDOW_AUTOSIZE);
    cv::Rect2d box;

    Mat frame;
    capture >> frame;
    box = selectROI("Frame", frame);
    MyTracker tracker = MyTracker();
    tracker.startTracking(argv[1], box, 1);
    while (!frame.empty()) {
        box = tracker.getNextPedestrianPosition();
        capture >> frame;
        rectangle(frame, box, cv::Scalar(0, 255, 0));
        imshow("Frame", frame);
        //exit
        int keyboard = waitKey(30);
        if (keyboard == 'q' || keyboard == 27)
            break;
    }
}
