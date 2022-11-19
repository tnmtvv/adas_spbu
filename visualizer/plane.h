#ifndef VISUALIZER_PLANE_H
#define VISUALIZER_PLANE_H
#include "models.h"
namespace models {

    class [[maybe_unused]] plane : public models{
    private:
        double width{};
        double length{};
        void initializateLocalPoints();
    public:

        plane(const cv::Vec3d& planeCoordinates, double width, double length);

        // Поменять длину и ширину плоскости
        void changeWidthAndLength(double width, double length);

        // Соеденить плоскости
        [[maybe_unused]] std::vector<models*> mergePlanes(plane* plane1);

        // Получить длину и ширину
        [[maybe_unused]] [[nodiscard]] double getWidth() const;

        [[maybe_unused]] [[nodiscard]] double getLength() const;

        // Генерация плавного поворота
        [[maybe_unused]] std::vector<models*> imposeRoad(plane* plane);
    };

} // models

#endif //VISUALIZER_PLANE_H
