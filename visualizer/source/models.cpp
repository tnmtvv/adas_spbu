#include "../include/models.h"

namespace models {

    // Перенести модель в мировых координатах
    void models::move(cv::Vec3d vector) {
        coordinateSystem->move(vector);
        for (int i = 0; i < localPoints.size(); i++){
            globalPoints[i].x += vector[0];
            globalPoints[i].y += vector[1];
            globalPoints[i].z += vector[2];
        }
    }

    // Перенести модель в локальных координатах
    void models::moveInLocalCoordinates(cv::Vec3d vector) {
        vector = coordinateSystem->moveToGlobalCoordinates(vector);
        coordinateSystem->setCoordinatesOfCenter(vector);
    }

    // Поворот модели
    void models::rotate(double angle, Axis axis) {
        coordinateSystem->rotate(angle, axis);
        globalPoints.clear();
        for (auto & localPoint : localPoints){
            globalPoints.emplace_back(coordinateSystem->moveToGlobalCoordinates(localPoint));
        }
    }

    // Получение точек модели в глобальных координатах
    std::vector<cv::Point3d>& models::getGlobalPoints() {
        return globalPoints;
    }

    models::models() {
        // Создание системы координат для модели
        coordinateSystem = std::make_shared<coordinateSystem::coordinateSystem>();
    }

    // Получение точек в локальной системе координат каждого объекта
    [[maybe_unused]] std::vector<cv::Point3d>& models::getLocalPoints() {
        return localPoints;
    }

    // Добавление точек к модели
    void models::addGlobalPoints(const cv::Vec3d& vec) {
        globalPoints.emplace_back(vec);
        localPoints.emplace_back(coordinateSystem->moveToLocalCoordinates(vec));
    }

    // Добавить вершины в список для сохранение в формате obj
    void models::setIndexes(const std::vector<int>& vector) {
        indexes.emplace_back(vector);
    }

    // Получить список вершин полигонов в формате obj
    std::vector<std::vector<int>> models::getIndexes() {
        return indexes;
    }

    // Получить координаты локальной системы координат объекта
    cv::Vec3d models::getCoordinatesOfCenter() {
        return this->coordinateSystem->getCoordinatesOfCenter();
    }

    std::shared_ptr<coordinateSystem::coordinateSystem> models::getCoordinateSystem() {
        return coordinateSystem;
    }
}