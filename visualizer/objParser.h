//
// Created by eeuri on 15.11.2022.
//

#ifndef VISUALIZER_OBJPARSER_H
#define VISUALIZER_OBJPARSER_H
#include <iostream>
#include "models.h"
namespace objParser {

    class [[maybe_unused]] objParser {
    public:
        [[maybe_unused]] static models::models* parse(const std::string& pathToFile);

        [[maybe_unused]] static void write(std::vector<models::models*> models, const std::string& pathToFile);
    };

} // objParser

#endif //VISUALIZER_OBJPARSER_H
