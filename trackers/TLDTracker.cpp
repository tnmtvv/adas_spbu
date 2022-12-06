#include <iostream>
#include "TLDTracker.h"
#include <opencv2/photo.hpp>
#include <opencv2/tracking/tracking_legacy.hpp>
void TLDTracker::startTracking(const std::string &path, cv::Rect2d pedestrian, int nFrame) {
    capture = cv::VideoCapture(path);
    pedestrianBox = pedestrian;
    tracker = cv::legacy::TrackerTLD::create();
    for (int i = 0; i < nFrame; i++) { capture >> frame; }
    denoise(frame);
    tracker->init(frame, pedestrianBox);
}

void TLDTracker::denoise(cv::Mat frame) {
    cv::fastNlMeansDenoising(frame, frame, 30, 7, 21);
}

TLDTracker::TLDTracker() = default;


cv::Rect2d TLDTracker::getNextPedestrianPosition() {
    capture >> frame;
    denoise(frame);
    if (!tracker->update(frame, pedestrianBox)) {
        std::cout << "failed csrt tracking" << std::endl;
    }
    return pedestrianBox;
}

void TLDTracker::reinit(cv::Rect2d boundingBox) {
    tracker->init(frame, boundingBox);
    pedestrianBox = boundingBox;
}