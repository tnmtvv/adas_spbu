#ifndef VISUALIZER_OBJPARSER_H
#define VISUALIZER_OBJPARSER_H

#include <iostream>
#include "models.h"
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

namespace objParser {
    class objParser {
    public:
        static std::shared_ptr<models::models> parse(const std::string& pathToFile);
        static void write(const std::vector<std::shared_ptr<models::models>>& models, const std::string& pathToFile);
    };
}

#endif
