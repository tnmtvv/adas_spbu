#ifndef VISUALIZER_CUBE_H
#define VISUALIZER_CUBE_H
#include "models.h"

namespace models {
    class cube : public models {
    public:
        cube(const cv::Vec3d& planeCoordinates, double width, double length, double height);
    };
}

#endif
