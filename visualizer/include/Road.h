#ifndef VISUALIZER_ROAD_H
#define VISUALIZER_ROAD_H

#include "models.h"
#include "plane.h"

namespace models {

    class [[maybe_unused]] Road {
    private:
        std::shared_ptr<StrategyOfAddBorders> strategy;
        std::vector<std::shared_ptr<plane>> planes;
    public:
        [[maybe_unused]] std::vector<std::vector<cv::Vec3d>> getRoadPoints(double stepOfYAxis);

        [[maybe_unused]] void addBorders(const std::shared_ptr<plane>& p, std::vector<cv::Point2d> vector);

        [[maybe_unused]] void setStrategy(std::shared_ptr<StrategyOfAddBorders> newStrategy);

        [[maybe_unused]] void IsValidBorders();
    };

}

#endif
