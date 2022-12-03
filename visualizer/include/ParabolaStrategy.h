#ifndef VISUALIZER_PARABOLASTRATEGY_H
#define VISUALIZER_PARABOLASTRATEGY_H

#include "StrategyOfAddBorders.h"
#include "parabola.h"

namespace models {

    class [[maybe_unused]] ParabolaStrategy : public StrategyOfAddBorders{
        public:
        std::shared_ptr<Borders> Strategy(std::vector<cv::Point2d> vector) override;
    };

} // models

#endif //VISUALIZER_PARABOLASTRATEGY_H
