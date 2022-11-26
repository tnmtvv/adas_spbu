#ifndef VISUALIZER_PARABOLASTRATEGY_H
#define VISUALIZER_PARABOLASTRATEGY_H

#include "StrategyOfAddBorders.h"
#include "parabola.h"

namespace models {

    class ParabolaStrategy : public StrategyOfAddBorders{
        public:
        std::vector<cv::Point3d> Strategy(std::vector<cv::Point2i> vector) override;
    };

} // models

#endif //VISUALIZER_PARABOLASTRATEGY_H
