#include "../include/ParabolaStrategy.h"
#include "../include/models.h"

namespace models {
    std::shared_ptr<Borders> ParabolaStrategy::Strategy(std::vector<cv::Point2d> vector) {
        return std::make_shared<parabola>(vector[0], vector[1], vector[2]);
    }
} // models