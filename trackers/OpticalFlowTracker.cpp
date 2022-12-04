
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include "OpticalFlowTracker.h"

using namespace cv;

OpticalFlowTracker::OpticalFlowTracker() = default;

Point2f getMean(std::vector<Point2f> &vec) {
    float sumX = 0;
    float sumY = 0;
    for (const Point2f &p: vec) {
        sumX += p.x;
        sumY += p.y;
    }
    float meanX = sumX / vec.size();
    float meanY = sumY / vec.size();
    return {meanX, meanY};

}

Point2i OpticalFlowTracker::getBoxMotion() {
    Point2f oldMean = getMean(oldFeatures);
    Point2f newMean = getMean(newFeatures);
    return newMean - oldMean;
}

cv::Rect2d getIntersection(Mat img, Rect2d box) {
    Point2d leftCorner = {max(0.0, box.x - box.width / 2), max(0.0, box.y - box.width / 2)};
    Point2d rightCorner = {min(double(img.rows), box.x + box.width / 2), min(double(img.cols), box.y + box.width / 2)};
    return Rect2d(leftCorner, Size(rightCorner.x - leftCorner.x, rightCorner.y - leftCorner.y));
}

void OpticalFlowTracker::updateBoxPosition() {
    Point2f boxMotion = getBoxMotion();
    pedestrianBox.x += boxMotion.x;
    pedestrianBox.y += boxMotion.y;
}

void OpticalFlowTracker::startTracking(std::string path, Rect2d pedestrian, int nFrame) {
    this->capture = VideoCapture(path);
    this->pedestrianBox = pedestrian;
    for (int i = 0; i < nFrame; i++){ capture >> oldFrame;}
    cvtColor(oldFrame, oldGray, COLOR_BGR2GRAY);
    goodFeaturesToTrack(oldGray(pedestrianBox), oldFeatures, featuresCount, 0.3, 7, Mat(), 7, false, 0.04);
}

Rect2d OpticalFlowTracker::getNextPedestrianPosition() {
    Mat newFrame, newGray;
    capture >> newFrame;
    cvtColor(newFrame, newGray, COLOR_BGR2GRAY);
    std::vector<uchar> status;
    std::vector<float> err;
    TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
    calcOpticalFlowPyrLK(oldGray(pedestrianBox), newGray(pedestrianBox), oldFeatures, newFeatures, status, err,
                         Size(15, 15), 2,
                         criteria);
    std::vector<Point2f> good_new = selectGoodFeatures(status, newFrame);
    updateBoxPosition();
    oldGray = newGray.clone();
    oldFeatures = good_new;
    if (oldFeatures.size() < 5) {
        goodFeaturesToTrack(newGray(pedestrianBox), oldFeatures, featuresCount, 0.3, 7, Mat(), 7, false, 0.04);
    }
    oldFrame = newFrame.clone();
    return pedestrianBox;
}

std::vector<Point2f> OpticalFlowTracker::selectGoodFeatures(std::vector<uchar> &status,
                                                            Mat &frame) {
    std::vector<Point2f> good_new;
    for (uint i = 0; i < oldFeatures.size(); i++) {
        if (status[i] == 1) {
            good_new.push_back(newFeatures[i]);
        }
    }
    return good_new;
}
