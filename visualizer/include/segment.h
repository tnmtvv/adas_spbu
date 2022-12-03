#ifndef VISUALIZER_SEGMENT_H
#define VISUALIZER_SEGMENT_H
#include "models.h"
#include "Borders.h"

namespace models {

    class [[maybe_unused]] segment: public Borders {
    private:
        [[maybe_unused]] double firstCoefficient{};
        [[maybe_unused]] double secondCoefficient{};
        [[nodiscard]] double countSegmentValue(double x) const;
        void calculateSegmentPoints(const cv::Point2d &firstPoint, const cv::Point2d &secondPoint);
    public:
        [[maybe_unused]] [[maybe_unused]] segment(const cv::Point2d& firstPoint, const cv::Point2d& secondPoint);

        [[maybe_unused]] [[maybe_unused]] [[nodiscard]] cv::Vec2d segmentCoefficient() const;
        double findBorderValue(double y) override;
        bool specificBorderIsValid() override;
    };

}
#endif
