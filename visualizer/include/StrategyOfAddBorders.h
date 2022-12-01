#ifndef VISUALIZER_STRATEGYOFADDBORDERS_H
#define VISUALIZER_STRATEGYOFADDBORDERS_H

#include <vector>
#include <opencv2/core/types.hpp>
namespace models {

    class StrategyOfAddBorders {
    public:
        virtual std::vector<cv::Point3d> Strategy(std::vector<cv::Point2d> vector) = 0;
    };

}

#endif
