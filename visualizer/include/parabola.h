#ifndef VISUALIZER_PARABOLA_H
#define VISUALIZER_PARABOLA_H

#include "models.h"
#include "Borders.h"
namespace models {
    class [[maybe_unused]] parabola : public Borders {
    private:
        [[maybe_unused]] double firstCoefficient{};
        [[maybe_unused]] double secondCoefficient{};
        [[maybe_unused]] double thirdCoefficient{};
        [[nodiscard]] double countParabolaValue(double x) const;
        void calculateParabolaPoints(const cv::Point2d &firstPoint, const cv::Point2d &secondPoint,
                                                const cv::Point2d &thirdPoint);
    public:
        [[maybe_unused]] parabola(const cv::Point2d& firstPoint, const cv::Point2d& secondPoint, const cv::Point2d& thirdPoint);
        [[maybe_unused]] [[nodiscard]] cv::Vec3d parabolaCoefficient() const;

        double findBorderValue(double y) override;
        bool specificBorderIsValid() override;
    };
}

#endif
