#ifndef OPTICALFLOWTRACKING_MYTRACKER_H
#define OPTICALFLOWTRACKING_MYTRACKER_H

#include <opencv2/videoio.hpp>
#include "trackers/Tracker.h"

class MyTracker : Tracker {
    cv::Mat currentFrame;
    std::vector<Tracker *> trackers;
    std::vector<double> weights;
    cv::VideoCapture capture;

    cv::Rect2d getMeanResult(std::vector<cv::Rect2d> &boundingBoxes);

public:

    MyTracker();

    void startTracking(const std::string &path, cv::Rect2d pedestrian, int nFrame) override;

    const cv::Rect2d getNextPedestrianPosition() override;

};

#endif //OPTICALFLOWTRACKING_MYTRACKER_H
