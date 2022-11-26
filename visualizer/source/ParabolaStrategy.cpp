//
// Created by eeuri on 26.11.2022.
//

#include "../include/ParabolaStrategy.h"

namespace models {
    std::vector<cv::Point3d> ParabolaStrategy::Strategy(std::vector<cv::Point2i> vector) {
        auto newParabola = std::make_shared<parabola>(vector[0], vector[1], vector[2]);
        return newParabola->getLocalPoints();
    }
} // models