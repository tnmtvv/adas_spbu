#ifndef VISUALIZER_OBJPARSER_H
#define VISUALIZER_OBJPARSER_H

#include <iostream>
#include "models.h"
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

namespace models {
    class [[maybe_unused]] objParser {
    public:
        [[maybe_unused]] static std::shared_ptr<models> parse(const std::string& pathToFile);

        [[maybe_unused]] static void write(const std::vector<std::shared_ptr<models>>& models, const std::string& pathToFile);
    };
}

#endif