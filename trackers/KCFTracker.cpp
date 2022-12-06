#include <iostream>
#include "KCFTracker.h"

KCFTracker::KCFTracker() = default;

void KCFTracker::startTracking(const std::string &path, cv::Rect2d pedestrian, int nFrame) {
    capture = cv::VideoCapture(path);
    pedestrianBox = pedestrian;
    tracker = cv::TrackerKCF::create();
    cv::Mat frame;
    for (int i = 0; i < nFrame; i++) { capture >> frame; }
    tracker->init(frame, pedestrianBox);
}

    cv::Rect2d KCFTracker::getNextPedestrianPosition() {
    cv::Mat frame;
    capture >> frame;
    if (!tracker->update(frame, pedestrianBox)) {
        std::cout << "failed kcf tracking" << std::endl;
    }
    return pedestrianBox;
}
