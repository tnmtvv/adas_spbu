//
// Created by eeuri on 15.11.2022.
//

#ifndef VISUALIZER_VERTEX_H
#define VISUALIZER_VERTEX_H
#include "models.h"

namespace models {

    class [[maybe_unused]] vertex {
    private:
        cv::Vec3d coordinates;
        cv::Vec3d normal;
    };

} // models

#endif //VISUALIZER_VERTEX_H
