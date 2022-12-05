#ifndef OPTICALFLOWTRACKING_TRACKER_H
#define OPTICALFLOWTRACKING_TRACKER_H

#include <opencv2/videoio.hpp>

class Tracker {
public:
    virtual void startTracking(const std::string &path, cv::Rect2d pedestrian, int nFrame) = 0;

    virtual const cv::Rect2d getNextPedestrianPosition() = 0;

    Tracker();

};

#endif //OPTICALFLOWTRACKING_TRACKER_H
