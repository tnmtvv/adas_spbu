#include "MyTracker.h"
#include "trackers/OpticalFlowTracker.h"
#include "trackers/KCFTracker.h"
#include "trackers/CSRTTracker.h"
#include "trackers/TLDTracker.h"
#include "trackers/GOTURNTracker.h"
#include <vector>
#include <numeric>
#include <iostream>
#include "consts.h"

MyTracker::MyTracker() {
    weightOfTracker.clear();
    trackers.clear();
    Tracker *opticalFlowTracker = new OpticalFlowTracker();
    Tracker *kcfTracker = new KCFTracker();
    Tracker *csrtTracker = new CSRTTracker();
    Tracker *tldTracker = new TLDTracker();
    Tracker *goturnTracker = new GOTURNTracker();

    trackers.push_back(opticalFlowTracker);
//    trackers.push_back(kcfTracker);
    trackers.push_back(csrtTracker);
    trackers.push_back(tldTracker);
    trackers.push_back(goturnTracker);

    weights.push_back(1);
//    weights.push_back(1);
    weights.push_back(1);
    weights.push_back(1);
    weights.push_back(0.5);
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

cv::Rect2d MyTracker::getNextPedestrianPosition() {
    static int nFrame = 0;
    nFrame++;
    std::vector<cv::Rect2d> boundingBoxes;
    for (auto tracker: trackers) {
        boundingBoxes.push_back(tracker->getNextPedestrianPosition());
    }
    cv::Rect2d newBoundingBox = getMeanResult(boundingBoxes);
    if (nFrame == framesBeforeReinitialization) {
        reinit(newBoundingBox);
        nFrame = 0;
    }
    return newBoundingBox;
}

void MyTracker::startTracking(const std::string &path, cv::Rect2d pedestrian, int nFrame) {
    for (auto tracker: trackers) {
        tracker->startTracking(path, pedestrian, nFrame);
    }
}

void MyTracker::reinit(cv::Rect2d boundingBox) {
    for (auto tracker: trackers) {
        tracker->reinit(boundingBox);
    }
}
