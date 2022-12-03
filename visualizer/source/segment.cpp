#include "../include/segment.h"

namespace models {
    [[maybe_unused]] segment::segment(const cv::Point2d &firstPoint, const cv::Point2d &secondPoint) {
        calculateSegmentPoints(firstPoint, secondPoint);

        for (auto & localPoint : localPoints)
        {
            globalPoints.emplace_back(coordinateSystem->moveToGlobalCoordinates(localPoint));
        }
    }

    void segment::calculateSegmentPoints(const cv::Point2d &firstPoint, const cv::Point2d &secondPoint) {
        // ax_1 + b = y1
        // ax_2 + b = y2

        cv::Mat deltaMat = (cv::Mat_<double>(2, 2) << firstPoint.x, 1, secondPoint.x, 1);

        cv::Mat deltaForFirstCoefficient = (cv::Mat_<double>(2, 2) << firstPoint.y, 1, secondPoint.y, 1);
        cv::Mat deltaForSecondCoefficient = (cv::Mat_<double>(2, 2) << firstPoint.x, firstPoint.y, secondPoint.x, secondPoint.y);

        auto denominator = cv::determinant(deltaMat);
        auto numeratorForFirstCoefficient = cv::determinant(deltaForFirstCoefficient);
        auto numeratorForSecondCoefficient = cv::determinant(deltaForSecondCoefficient);

        firstCoefficient = numeratorForFirstCoefficient / denominator;
        secondCoefficient = numeratorForSecondCoefficient / denominator;

        auto minXCoordinates = fmin(secondPoint.x, firstPoint.x);
        auto maxXCoordinates = fmax(secondPoint.x,firstPoint.x);

        if (countSegmentValue(minXCoordinates) < countSegmentValue(maxXCoordinates)) {
            localPoints.emplace_back(minXCoordinates, countSegmentValue(minXCoordinates), 0);
            localPoints.emplace_back(maxXCoordinates, countSegmentValue(maxXCoordinates), 0);
        }
        else {
            localPoints.emplace_back(maxXCoordinates, countSegmentValue(maxXCoordinates), 0);
            localPoints.emplace_back(minXCoordinates, countSegmentValue(minXCoordinates), 0);
        }

        /*
        double step = 1;
        auto currentXCoordinate = minXCoordinates;

        while (currentXCoordinate < maxXCoordinates) {
            localPoints.emplace_back(currentXCoordinate, countSegmentValue(currentXCoordinate), 0);
            currentXCoordinate += step;
        }

        if (abs(localPoints.back().x - maxXCoordinates) > 0.01) {
            localPoints.emplace_back(maxXCoordinates, countSegmentValue(maxXCoordinates), 0);
        }*/
    }

    double segment::countSegmentValue(double x) const {
        return firstCoefficient * x + secondCoefficient;
    }

    double segment::findBorderValue(double y) {
        return (y - secondCoefficient) / firstCoefficient;
    }

    [[maybe_unused]] cv::Vec2d segment::segmentCoefficient() const {
        return {firstCoefficient, secondCoefficient};
    }

    bool segment::specificBorderIsValid() {
        return false;
    }
} // models