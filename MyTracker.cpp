#include "MyTracker.h"
#include "trackers/OpticalFlowTracker.h"
#include "trackers/KCFTracker.h"
#include "trackers/CSRTTracker.h"
#include <vector>
#include <numeric>
#include <iostream>

MyTracker::MyTracker() {
    Tracker *opticalFlowTracker = new OpticalFlowTracker();
    Tracker *kcfTracker = new KCFTracker();
    Tracker *csrtTracker = new CSRTTracker();
    trackers.push_back(opticalFlowTracker);
    trackers.push_back(kcfTracker);
    trackers.push_back(csrtTracker);
    weights.push_back(1);
    weights.push_back(0);
    weights.push_back(1);
}

cv::Rect2d MyTracker::getMeanResult(std::vector<cv::Rect2d> &boundingBoxes) {
    cv::Point2d avgCenter = {0, 0};
    cv::Point2d avgParams = {0, 0};
    double sumWeight = std::reduce(weights.begin(), weights.end());
    for (int i = 0; i < boundingBoxes.size(); i++) {
        avgCenter += {boundingBoxes[i].x * weights[i], boundingBoxes[i].y * weights[i]};
        avgParams += {boundingBoxes[i].width * weights[i], boundingBoxes[i].height * weights[i]};
    }
    avgCenter.x /= sumWeight;
    avgCenter.y /= sumWeight;
    avgParams.x /= sumWeight;
    avgParams.y /= sumWeight;
    return cv::Rect2d(avgCenter, cv::Size(avgParams.x, avgParams.y));
}

const cv::Rect2d MyTracker::getNextPedestrianPosition() {
    std::vector<cv::Rect2d> boundingBoxes;
    for (auto tracker: trackers) {
        boundingBoxes.push_back(tracker->getNextPedestrianPosition());
    }
    return getMeanResult(boundingBoxes);
}

void MyTracker::startTracking(const std::string &path, cv::Rect2d pedestrian, int nFrame) {
    for (auto tracker: trackers) {
        tracker->startTracking(path, pedestrian, nFrame);
    }
}
