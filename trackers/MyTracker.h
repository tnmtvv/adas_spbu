#ifndef OPTICALFLOWTRACKING_MYTRACKER_H
#define OPTICALFLOWTRACKING_MYTRACKER_H

#include <opencv2/videoio.hpp>
#include <map>
#include "trackers/Tracker.h"

class MyTracker : Tracker {
    cv::Mat currentFrame;
    std::vector<Tracker *> trackers;
    std::vector<double> weights;
    cv::VideoCapture capture;
    std::map<Tracker *, double> weightOfTracker;

    cv::Rect2d getMeanResult(std::vector<cv::Rect2d> &boundingBoxes);

public:

    MyTracker();
    //reinit every tracker by bounding box
    void reinit(cv::Rect2d boundingBox);

    //tracker initialization. Path - path to video, bounding box, nFrame - number of frame where from tracking should start
    void startTracking(const std::string &path, cv::Rect2d pedestrian, int nFrame) override;

    //gives mean result from several trackers
    cv::Rect2d getNextPedestrianPosition() override;

};

#endif //OPTICALFLOWTRACKING_MYTRACKER_H
