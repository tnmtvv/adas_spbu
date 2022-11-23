#ifndef VISUALIZER_CUBE_H
#define VISUALIZER_CUBE_H

#include "models.h"

namespace models {
    class [[maybe_unused]] cube : public models {
    public:
        [[maybe_unused]] cube(const cv::Vec3d& planeCoordinates, double width, double length, double height);
    };
}

#endif
