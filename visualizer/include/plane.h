#ifndef VISUALIZER_PLANE_H
#define VISUALIZER_PLANE_H

#include "models.h"
#include "StrategyOfAddBorders.h"
#include "Borders.h"
namespace models {
    class [[maybe_unused]] plane : public models{
    private:
        double width;
        double length;
        void initializateLocalPoints();
        std::vector<std::shared_ptr<Borders>> leftBorderModels;
        std::vector<std::shared_ptr<Borders>> rightBorderModels;
    public:

        plane(const cv::Vec3d& planeCoordinates, double width, double length);

        // Поменять длину и ширину плоскости
        [[maybe_unused]] void changeWidthAndLength(double width, double length);

        // Соеденить плоскости
        [[maybe_unused]] std::vector<std::shared_ptr<plane>> mergePlanes(coordinateSystem::Axis axis, double maxAngle, double stepLength, double stepAngle);

        // Получить длину и ширину
        [[maybe_unused]] [[nodiscard]] double getWidth() const;

        [[maybe_unused]] [[nodiscard]] double getLength() const;

        [[maybe_unused]] void addLeftBorder(std::shared_ptr<Borders>);
        [[maybe_unused]] void addRightBorder(std::shared_ptr<Borders>);


        /*
        [[maybe_unused]] std::vector<cv::Point3d> getLeftBorder();

        std::vector<cv::Point3d> getRightBorder();*/

        std::vector<std::shared_ptr<Borders>> getRightBorderModels();
        std::vector<std::shared_ptr<Borders>> getLeftBorderModels();
    };
}

#endif