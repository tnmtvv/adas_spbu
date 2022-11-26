#include "../include/objParser.h"

namespace models {

    [[maybe_unused]] std::shared_ptr<models> objParser::parse(const std::string& pathToFile) {
        std::ifstream infile(pathToFile);
        std::string line;
        auto model = std::make_shared<models>();
        while (std::getline(infile, line))
        {
            // не умею работать с нормалями к вершинам
            auto substring = line.substr(0,2);
            if (substring == "v "){
                std::istringstream coordinates(line.substr(2));
                double firstCoordinates, secondCoordinates, thirdCoordinates;
                coordinates >> firstCoordinates >> secondCoordinates >> thirdCoordinates;
                cv::Vec3d vectorOfCoordinates(firstCoordinates, secondCoordinates, thirdCoordinates);
                model->addGlobalPoints(vectorOfCoordinates);
            }
            else if(substring == "f " || substring == "l ") {
                std::istringstream curveIndexesStream(line.substr(1));
                std::string token;
                std::vector<int> indexes;
                while(std::getline(curveIndexesStream, token, ' '))
                {
                    curveIndexesStream >> token;
                    std::istringstream tokenStream(token);
                    std::string index;
                    if (std::getline(tokenStream, index, '/'))
                    {
                        int pointIndex;
                        std::istringstream pointIndexStream(index);
                        pointIndexStream>>pointIndex;
                        indexes.emplace_back(pointIndex - 1);
                    }
                }

                model->indexes.emplace_back(indexes);
            }
        }
        infile.close();
        return model;
    }

    [[maybe_unused]] void objParser::write(const std::vector<std::shared_ptr<models>>& models, const std::string& pathToFile) {
        std::ofstream infile(pathToFile);
        int t = 0;
        unsigned long l = 0;
        for(auto & model : models) {
            infile << "o "<<t<<std::endl;
            auto globalPoints = model->getGlobalPoints();
            for (auto & globalPoint : globalPoints) {
                infile << "v " << globalPoint.x << " " << globalPoint.y << " " << globalPoint.z
                       << std::endl;
            }

            auto polygons = model->getIndexes();
            for (auto & polygon : polygons) {
                infile << "f";
                for (int h : polygon) {
                    infile << " " << h + 1 + l;
                }
                infile << std::endl;
            }

            l += globalPoints.size();
            t++;
        }
        infile.close();
    }
}