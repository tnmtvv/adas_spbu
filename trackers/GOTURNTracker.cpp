#include <iostream>
#include "GOTURNTracker.h"

void GOTURNTracker::startTracking(const std::string &path, cv::Rect2d pedestrian, int nFrame) {
    capture = cv::VideoCapture(path);
    pedestrianBox = pedestrian;
    tracker = cv::TrackerGOTURN::create();
    for (int i = 0; i < nFrame; i++) { capture >> frame; }
    denoise(frame);
    tracker->init(frame, pedestrianBox);
}

cv::Rect2d GOTURNTracker::getNextPedestrianPosition() {
    capture >> frame;
    denoise(frame);
    if (!tracker->update(frame, pedestrianBox)) {
        std::cout << "failed goturn tracking" << std::endl;
    }
    return pedestrianBox;
}

void GOTURNTracker::denoise(cv::Mat frame) {

}

void GOTURNTracker::reinit(cv::Rect2d boundingBox) {
    tracker->init(frame, boundingBox);
}

GOTURNTracker::GOTURNTracker() = default;
