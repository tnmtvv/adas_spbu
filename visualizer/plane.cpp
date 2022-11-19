//
// Created by eeuri on 13.11.2022.
//

#include "plane.h"
#include "opencv2/opencv.hpp"

namespace models {
    plane::plane(const cv::Vec3d& planeCoordinates, double width, double length) : models() {
        move(planeCoordinates);
        changeWidthAndLength(width, length);
    }

    void plane::changeWidthAndLength(double newWidth, double newLength) {
        width = newWidth;
        length = newLength;
        initializateLocalPoints();
    }

    void plane::initializateLocalPoints() {
        cv::Point3d firstPoint(width / 2, length / 2, 0);
        cv::Point3d secondPoint(width / 2, -length / 2, 0);
        cv::Point3d thirdPoint(-width / 2, length / 2, 0);
        cv::Point3d fourthPoint(-width / 2, -length / 2, 0);

        localPoints.clear();
        globalPoints.clear();

        // Добавляем координаты
        this->localPoints.emplace_back(firstPoint);
        this->localPoints.emplace_back(secondPoint);
        this->localPoints.emplace_back(thirdPoint);
        this->localPoints.emplace_back(fourthPoint);

        // Вершины нужных нам полигонов
        indexes.emplace_back(std::vector<int>{0, 1, 2});
        indexes.emplace_back(std::vector<int>{1, 2, 3});

        // Считаем глобальные координаты
        for (auto & localPoint : localPoints)
        {
            globalPoints.emplace_back(coordinateSystem->moveToGlobalCoordinates(localPoint));
        }
    }

    [[maybe_unused]] std::vector<models*> plane::mergePlanes(plane *plane1) {
        /*
            Проверка на равенство углов
        */
        std::vector<models*> models;
        models.emplace_back(this);
        plane* tempPlane = this;
        // Тут на xAxis, но можно легко обобщить
        while (tempPlane->coordinateSystem->getAngle(xAxis) > plane1->coordinateSystem->getAngle(xAxis))
        {
            auto angle = tempPlane->coordinateSystem->getAngle(xAxis);
            auto lol = tempPlane->coordinateSystem->getCoordinatesOfCenter();

            // шаг по длине и углу можно принимать как аргумент
            double len = 2;
            auto y = tempPlane->length/ 2 + len / 2;
            auto nextPlane = tempPlane->coordinateSystem->moveToGlobalCoordinates(cv::Vec3d(0, y, 0));
            auto* newPlane = new plane(cv::Vec3d(nextPlane), tempPlane->width, len);
            angle -= CV_PI / 90;
            newPlane->rotate(angle, xAxis);
            models.emplace_back(newPlane);
            tempPlane = newPlane;
        }
        auto i = tempPlane->length/ 2 + plane1->length / 2;
        auto dots = tempPlane->coordinateSystem->moveToGlobalCoordinates(cv::Vec3d(0, i, 0));
        auto nextPlane = new plane(dots, tempPlane->width, plane1->length);
        nextPlane->rotate(plane1->coordinateSystem->getAngle(xAxis), xAxis);
        models.emplace_back(nextPlane);
        return models;
    }

    [[maybe_unused]] double plane::getWidth() const {
        return width;
    }

    [[maybe_unused]] double plane::getLength() const {
        return length;
    }

    // ----------------------Копипаст верхней функции
    [[maybe_unused]] std::vector<models*> plane::imposeRoad(plane* plane1) {
        std::vector<models*> models;
        models.emplace_back(this);
        plane* tempPlane = this;
        while (tempPlane->coordinateSystem->getAngle(zAxis) < plane1->coordinateSystem->getAngle(zAxis))
        {
            auto q = tempPlane->coordinateSystem->getAngle(zAxis);
            auto lol = tempPlane->coordinateSystem->getCoordinatesOfCenter();
            double len = 2;
            auto y = tempPlane->length/ 2;
            auto ef = tempPlane->coordinateSystem->moveToGlobalCoordinates(cv::Vec3d(0, y, 0));
            auto* newPlane = new plane(cv::Vec3d(ef), tempPlane->width, len);
            newPlane->rotate(tempPlane->coordinateSystem->getAngle(xAxis), xAxis);

            q += CV_PI / 90;
            newPlane->rotate(q, zAxis);
            models.emplace_back(newPlane);
            tempPlane = newPlane;
        }
        auto i = tempPlane->length/ 2 + plane1->length / 2;
        auto dots = tempPlane->coordinateSystem->moveToGlobalCoordinates(cv::Vec3d(0, i, 0));
        auto ef = new plane(dots, tempPlane->width, plane1->length);
        ef->rotate(tempPlane->coordinateSystem->getAngle(xAxis), xAxis);

        ef->rotate(tempPlane->coordinateSystem->getAngle(zAxis), zAxis);

        models.emplace_back(ef);
        return models;
    }

} // models