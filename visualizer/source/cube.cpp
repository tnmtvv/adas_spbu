#include "../include/cube.h"

namespace models {
    cube::cube(const cv::Vec3d& planeCoordinates, double width, double length, double height): models() {
        this->move(planeCoordinates);
        cv::Point3d firstPoint(width / 2, length / 2, height / 2);
        cv::Point3d secondPoint(width / 2, -length / 2, height / 2);
        cv::Point3d thirdPoint(width / 2, length / 2, -height / 2);
        cv::Point3d fourthPoint(width / 2, -length / 2, -height / 2);

        cv::Point3d fifthPoint(-width/2, length / 2, height / 2);
        cv::Point3d sixthPoint(-width/2, -length / 2, height / 2);
        cv::Point3d seventhPoint(-width/2, length / 2, -height / 2);
        cv::Point3d eightPoint(-width/2, -length / 2, -height / 2);

        this->localPoints.emplace_back(firstPoint);
        this->localPoints.emplace_back(secondPoint);
        this->localPoints.emplace_back(thirdPoint);
        this->localPoints.emplace_back(fourthPoint);
        this->localPoints.emplace_back(fifthPoint);
        this->localPoints.emplace_back(sixthPoint);
        this->localPoints.emplace_back(seventhPoint);
        this->localPoints.emplace_back(eightPoint);

        indexes.emplace_back(std::vector<int>{0, 1, 2});
        indexes.emplace_back(std::vector<int>{1, 2, 3});
        indexes.emplace_back(std::vector<int>{4, 5, 6});
        indexes.emplace_back(std::vector<int>{5, 6, 7});
        indexes.emplace_back(std::vector<int>{0, 2, 6});
        indexes.emplace_back(std::vector<int>{0, 4, 6});
        indexes.emplace_back(std::vector<int>{1, 3, 7});
        indexes.emplace_back(std::vector<int>{1, 5, 7});
        indexes.emplace_back(std::vector<int>{0, 1, 7});

        for (auto & localPoint : localPoints)
        {
            globalPoints.emplace_back(coordinateSystem->moveToGlobalCoordinates(localPoint));
        }
    }
}