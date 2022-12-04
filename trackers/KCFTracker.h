#ifndef OPTICALFLOWTRACKING_KCFTRACKER_H
#define OPTICALFLOWTRACKING_KCFTRACKER_H


#include <opencv2/tracking.hpp>
#include "Tracker.h"

class KCFTracker: public Tracker {
    cv::VideoCapture capture;
    cv::Rect2i pedestrianBox;
    cv::Ptr<cv::TrackerKCF> tracker;
public:
    KCFTracker();
    void startTracking(std::string path, cv::Rect2d pedestrian, int nFrame) override;
    cv::Rect2d getNextPedestrianPosition() override;
};


#endif //OPTICALFLOWTRACKING_KCFTRACKER_H
