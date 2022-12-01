#ifndef VISUALIZER_PARABOLA_H
#define VISUALIZER_PARABOLA_H

#include "models.h"

namespace models {
    class [[maybe_unused]] parabola : public models {
    private:
        [[maybe_unused]] double firstCoefficient{};
        [[maybe_unused]] double secondCoefficient{};
        [[maybe_unused]] double thirdCoefficient{};
        [[nodiscard]] double countParabolaValue(double x) const;
        void calculateParabolaPoints(const cv::Point2d &firstPoint, const cv::Point2d &secondPoint,
                                                const cv::Point2d &thirdPoint);
        [[maybe_unused]] std::vector<cv::Point3d> borderPoints;
    public:
        [[maybe_unused]] parabola(const cv::Point2d& firstPoint, const cv::Point2d& secondPoint, const cv::Point2d& thirdPoint);
        [[maybe_unused]] [[nodiscard]] cv::Vec3d parabolaCoefficient() const;
    };
}

#endif
