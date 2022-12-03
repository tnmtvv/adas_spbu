#ifndef VISUALIZER_STRATEGYOFADDBORDERS_H
#define VISUALIZER_STRATEGYOFADDBORDERS_H

#include <vector>
#include <opencv2/core/types.hpp>
#include "Borders.h"
namespace models {

    class StrategyOfAddBorders {
    public:
        virtual std::shared_ptr<Borders> Strategy(std::vector<cv::Point2d> vector) = 0;
    };

}

#endif
