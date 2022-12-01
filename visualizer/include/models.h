#ifndef VISUALIZER_MODELS_H
#define VISUALIZER_MODELS_H

#include "coordinateSystem.h"

namespace models {
    class models {
        friend class objParser;
    protected:
        // Система координат модели
        std::shared_ptr<coordinateSystem::coordinateSystem> coordinateSystem;

        // Точки модели в локальной системе координат
        std::vector<cv::Point3d> localPoints;

        // Точки модели в глобальной системе координат
        std::vector<cv::Point3d> globalPoints;

        // Список вершин полиогонов для хранение в формате .obj
        std::vector<std::vector<int>> indexes;

    public:
        void move(cv::Vec3d vector);
        std::shared_ptr<coordinateSystem::coordinateSystem> getCoordinateSystem();
        void rotate(double angle, Axis axis);
        std::vector<cv::Point3d>& getGlobalPoints();
        void addGlobalPoints(const cv::Vec3d& vec);

        [[maybe_unused]] std::vector<cv::Point3d>& getLocalPoints();
        std::vector<std::vector<int>> getIndexes();
        cv::Vec3d getCoordinatesOfCenter();
        models();
        void moveInLocalCoordinates(cv::Vec3d vector);

        void setCoordinatesOfCenter(cv::Vec3d vec);
    };
}

#endif
