#ifndef TRACKING_GOTURNTRACKER_H
#define TRACKING_GOTURNTRACKER_H

#include <opencv2/tracking.hpp>
#include "Tracker.h"

class GOTURNTracker : public Tracker {
    cv::VideoCapture capture;
    cv::Rect2i pedestrianBox;
    cv::Ptr<cv::TrackerGOTURN> tracker;

    void denoise(cv::Mat frame);

public:
    GOTURNTracker();

    void startTracking(const std::string &path, cv::Rect2d pedestrian, int nFrame) override;

    cv::Rect2d getNextPedestrianPosition() override;
};


#endif //TRACKING_GOTURNTRACKER_H
