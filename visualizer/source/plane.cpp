#include <utility>

#include "../include/plane.h"
#include "opencv2/opencv.hpp"

namespace models {
    plane::plane(const cv::Vec3d& planeCoordinates, double newWidth, double newLength) : models() {
        move(planeCoordinates);
        width = newWidth;
        length = newLength;
        initializateLocalPoints();
    }

    void plane::changeWidthAndLength(double newWidth, double newLength) {
        width = newWidth;
        length = newLength;
        initializateLocalPoints();
    }

    void plane::initializateLocalPoints() {
        localPoints.clear();
        globalPoints.clear();

        // Добавляем координаты
        this->localPoints.emplace_back(width / 2, length / 2, 0);
        this->localPoints.emplace_back(width / 2, -length / 2, 0);
        this->localPoints.emplace_back(-width / 2, length / 2, 0);
        this->localPoints.emplace_back(-width / 2, -length / 2, 0);

        // Вершины нужных нам полигонов
        indexes.emplace_back(std::vector<int>{0, 1, 2});
        indexes.emplace_back(std::vector<int>{1, 2, 3});

        // Считаем глобальные координаты
        for (auto & localPoint : localPoints)
        {
            globalPoints.emplace_back(coordinateSystem->moveToGlobalCoordinates(localPoint));
        }
    }

    std::vector<std::shared_ptr<plane>> plane::mergePlanes(Axis axis, double maxAngle, double stepLength,
                                                           double stepAngle)
    {
        std::vector<std::shared_ptr<plane>> planes;
        planes.emplace_back(this);
        std::shared_ptr<plane> tempPlane = std::make_shared<plane>(this->getCoordinatesOfCenter(),
                                                                   this->width, this->length);

        while (tempPlane->coordinateSystem->getAngle(axis) < maxAngle)
        {
            auto coordinatesOfCenterForNextPlane = tempPlane->coordinateSystem->
                    moveToGlobalCoordinates(cv::Vec3d(0, tempPlane->length/ 2 + stepLength / 2, 0));
            std::shared_ptr<plane> nextPlane = std::make_shared<plane>
                    (cv::Vec3d(coordinatesOfCenterForNextPlane),
                     tempPlane->width, stepLength);
            nextPlane->rotate(tempPlane->coordinateSystem->getAngle(axis) + stepAngle, axis);
            planes.emplace_back(nextPlane);
            tempPlane = nextPlane;
        }

        return planes;
    }

    double plane::getWidth() const {
        return width;
    }

    double plane::getLength() const {
        return length;
    }

    /*
    std::vector<std::shared_ptr<plane>> plane::createCrossRoad(const std::shared_ptr<plane>& plane1){
        auto vec = imposeRoad(plane1, 2, CV_PI/ 90);
        auto lastPlane = vec[vec.size() - 1];
        auto firstPlane = vec[0];
        lastPlane->rotate(-CV_PI, zAxis);
        auto newVector = lastPlane->imposeRoad(firstPlane, 8, CV_PI/90);
        vec[vec.size() - 1]->rotate(CV_PI, zAxis);
        auto index = vec.size() - 1;
        vec.insert( vec.end(), newVector.begin(), newVector.end() );/*
        lastPlane = vec[vec.size() - 1];
        firstPlane = vec[index];
        firstPlane->rotate(CV_PI, zAxis);
        lastPlane->rotate(CV_PI, zAxis);
        newVector = lastPlane->imposeRoad(firstPlane, 2);
        index = vec.size() - 1;
        vec.insert( vec.end(), newVector.begin(), newVector.end() );
        lastPlane = vec[vec.size() - 1];
        firstPlane = vec[index];
        return vec;
    }

    // ----------------------Копипаст верхней функции
    [[maybe_unused]] std::vector<std::shared_ptr<plane>> plane::imposeRoad(const std::shared_ptr<plane>& plane1, double len, double angle) {
        std::vector<std::shared_ptr<plane>> models;
        models.emplace_back(this);
        plane* tempPlane = this;
        while (tempPlane->coordinateSystem->getAngle(zAxis) < plane1->coordinateSystem->getAngle(zAxis))
        {
            auto kik = plane1->coordinateSystem->getAngle(zAxis);
            auto q = tempPlane->coordinateSystem->getAngle(zAxis);
            auto lol = tempPlane->coordinateSystem->getCoordinatesOfCenter();
            auto y = tempPlane->length/ 2;
            auto ef = tempPlane->coordinateSystem->moveToGlobalCoordinates(cv::Vec3d(0, y, 0));
            std::cout<<ef;
            auto* newPlane = new plane(cv::Vec3d(ef), tempPlane->width, len);
            newPlane->rotate(tempPlane->coordinateSystem->getAngle(xAxis), xAxis);

            q += angle;
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
    } */
}