#include <iostream>
#include "CSRTTracker.h"
#include <opencv2/photo.hpp>

void CSRTTracker::startTracking(const std::string &path, cv::Rect2d pedestrian, int nFrame) {
    capture = cv::VideoCapture(path);
    pedestrianBox = pedestrian;
    tracker = cv::TrackerCSRT::create();
    for (int i = 0; i < nFrame; i++) { capture >> frame; }
    denoise(frame);
    tracker->init(frame, pedestrianBox);

}

void CSRTTracker::denoise(cv::Mat frame) {
    cv::fastNlMeansDenoising(frame, frame, 30, 7, 21);
}

CSRTTracker::CSRTTracker() = default;

cv::Rect2d CSRTTracker::getNextPedestrianPosition() {
    capture >> frame;
    denoise(frame);
    if (!tracker->update(frame, pedestrianBox)) {
        std::cout << "failed csrt tracking" << std::endl;
    }
    return pedestrianBox;
}

void CSRTTracker::reinit(cv::Rect2d boundingBox) {
    tracker->init(frame, boundingBox);
    pedestrianBox = boundingBox;

}