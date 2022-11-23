#include "../include/cube.h"

namespace models {

    // бесполезный для построения дороги класс, используется для проверки работы поворотов и сдвигов моделей
    [[maybe_unused]] cube::cube(const cv::Vec3d& planeCoordinates, double width, double length, double height): models() {
        this->move(planeCoordinates);

        this->localPoints.emplace_back(width / 2, length / 2, height / 2);
        this->localPoints.emplace_back(width / 2, -length / 2, height / 2);
        this->localPoints.emplace_back(width / 2, length / 2, -height / 2);
        this->localPoints.emplace_back(width / 2, -length / 2, -height / 2);
        this->localPoints.emplace_back(-width/2, length / 2, height / 2);
        this->localPoints.emplace_back(-width/2, -length / 2, height / 2);
        this->localPoints.emplace_back(-width/2, length / 2, -height / 2);
        this->localPoints.emplace_back(-width/2, -length / 2, -height / 2);

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