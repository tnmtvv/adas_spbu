#include "../include/parabola.h"

namespace models {

    double parabola::countParabolaValue(double x) const {
        return firstCoefficient * x * x + secondCoefficient * x + thirdCoefficient;
    }

    void parabola::calculateParabolaPoints(const cv::Point2i& firstPoint, const cv::Point2i& secondPoint, const cv::Point2i& thirdPoint) {

        // ax_1^2 + bx_1 + c = y1
        // ax_2^2 + bx_2 + c = y2
        // ax_3^2 + bx_3 + c = y3

        int delta[3][3] = {{firstPoint.x * firstPoint.x, firstPoint.x, 1},
                           {secondPoint.x * secondPoint.x, secondPoint.x, 1},
                           {thirdPoint.x * thirdPoint.x, thirdPoint.x, 1}};
        cv::Mat deltaMat(3, 3, CV_64FC1, delta);

        int deltaA[3][3] = {{firstPoint.y, firstPoint.x, 1},
                            {secondPoint.y, secondPoint.x, 1},
                            {thirdPoint.y, thirdPoint.x, 1}};
        cv::Mat deltaForFirstCoefficient(3, 3, CV_64FC1, deltaA);

        int deltaB[3][3] = {{firstPoint.x * firstPoint.x, firstPoint.y, 1},
                            {secondPoint.x * secondPoint.x, secondPoint.y, 1},
                            {thirdPoint.x * thirdPoint.x, thirdPoint.y, 1}};
        cv::Mat deltaForSecondCoefficient(3, 3, CV_64FC1, deltaB);

        int deltaC[3][3] = {{firstPoint.x * firstPoint.x, firstPoint.x, firstPoint.y},
                            {secondPoint.x * secondPoint.x, secondPoint.x, firstPoint.y},
                            {thirdPoint.x * thirdPoint.x, thirdPoint.x, firstPoint.y}};
        cv::Mat deltaForThirdCoefficient(3, 3, CV_64FC1, deltaC);

        auto denominator = cv::determinant(deltaMat);
        auto numeratorForFirstCoefficient = cv::determinant(deltaForFirstCoefficient);
        auto numeratorForSecondCoefficient = cv::determinant(deltaForSecondCoefficient);
        auto numeratorForThirdCoefficient = cv::determinant(deltaForThirdCoefficient);

        firstCoefficient = numeratorForFirstCoefficient / denominator;
        secondCoefficient = numeratorForSecondCoefficient / denominator;
        thirdCoefficient = numeratorForThirdCoefficient / denominator;

        auto minXCoordinates = fmin(fmin(secondPoint.x, thirdPoint.x), firstPoint.x);
        auto maxXCoordinates = fmax(fmax(secondPoint.x, thirdPoint.x), firstPoint.x);

        double step = 0.5;
        auto currentXCoordinate = minXCoordinates;
        while (currentXCoordinate < maxXCoordinates) {
            localPoints.emplace_back(currentXCoordinate, countParabolaValue(minXCoordinates), 0);
            minXCoordinates += step;
        }

        if (abs(localPoints.back().x - maxXCoordinates) > 0.01) {
            localPoints.emplace_back(maxXCoordinates, countParabolaValue(maxXCoordinates), 0);
        }
    }

    [[maybe_unused]] parabola::parabola(const cv::Point2i& firstPoint, const cv::Point2i& secondPoint, const cv::Point2i& thirdPoint) {
        calculateParabolaPoints(firstPoint, secondPoint, thirdPoint);

        for (auto & localPoint : localPoints)
        {
            globalPoints.emplace_back(coordinateSystem->moveToGlobalCoordinates(localPoint));
        }
    }

    [[maybe_unused]] cv::Vec3d parabola::parabolaCoefficient() const {
        return {firstCoefficient, secondCoefficient, thirdCoefficient};
    }
}
