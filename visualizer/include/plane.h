#ifndef VISUALIZER_PLANE_H
#define VISUALIZER_PLANE_H

#include "models.h"
#include "StrategyOfAddBorders.h"

namespace models {
    class [[maybe_unused]] plane : public models{
    private:
        double width;
        double length;
        void initializateLocalPoints();
        std::shared_ptr<StrategyOfAddBorders> strategy;
        std::vector<cv::Point3d> leftBorder;
        std::vector<cv::Point3d> rightBorder;
    public:

        plane(const cv::Vec3d& planeCoordinates, double width, double length);

        // Поменять длину и ширину плоскости
        [[maybe_unused]] void changeWidthAndLength(double width, double length);

        // Соеденить плоскости
        [[maybe_unused]] std::vector<std::shared_ptr<plane>> mergePlanes(Axis axis, double maxAngle, double stepLength, double stepAngle);

        // Получить длину и ширину
        [[maybe_unused]] [[nodiscard]] double getWidth() const;

        [[maybe_unused]] [[nodiscard]] double getLength() const;

        [[maybe_unused]] void addBorders(std::vector<cv::Point2d> vector);

        [[maybe_unused]] void setStrategy(std::shared_ptr<StrategyOfAddBorders> newStrategy);

        [[maybe_unused]] std::vector<cv::Point3d> getLeftBorder();

        std::vector<cv::Point3d> getRightBorder();
    };
}

#endif