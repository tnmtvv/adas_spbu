#ifndef VISUALIZER_PLANE_H
#define VISUALIZER_PLANE_H

#include "models.h"

namespace models {
    class plane : public models{
    private:
        double width;
        double length;
        void initializateLocalPoints();
    public:

        plane(const cv::Vec3d& planeCoordinates, double width, double length);

        // Поменять длину и ширину плоскости
        void changeWidthAndLength(double width, double length);

        // Соеденить плоскости
        std::vector<std::shared_ptr<plane>> mergePlanes(const std::shared_ptr<plane>& plane1);

        // Получить длину и ширину
        [[nodiscard]] double getWidth() const;

        [[nodiscard]] double getLength() const;

        // Генерация плавного поворота
        std::vector<std::shared_ptr<plane>> imposeRoad(const std::shared_ptr<plane>& plane, double len, double angle);

        std::vector<std::shared_ptr<plane>> createCrossRoad(const std::shared_ptr<plane>& plane1);
    };
}

#endif