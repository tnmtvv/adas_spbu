
#ifndef TRACKING_TLDTRACKER_H
#define TRACKING_TLDTRACKER_H
#include <opencv2/tracking.hpp>
#include "Tracker.h"
#include <opencv2/tracking/tracking_legacy.hpp>

class TLDTracker : public Tracker{
    cv::VideoCapture capture;
    cv::Rect_<double> pedestrianBox;
    cv::Ptr<cv::legacy::TrackerTLD> tracker;
    void denoise(cv::Mat frame);
public:
    TLDTracker();
    void startTracking(const std::string& path, cv::Rect_<double> pedestrian, int nFrame) override;
    cv::Rect2d getNextPedestrianPosition() override;
};


#endif //TRACKING_TLDTRACKER_H
