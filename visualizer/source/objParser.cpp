#include "../include/objParser.h"

namespace objParser {

    std::shared_ptr<models::models> objParser::parse(const std::string& pathToFile) {
        std::ifstream infile(pathToFile);
        std::string line;
        auto model = std::make_shared<models::models>();
        while (std::getline(infile, line))
        {
            // пока не умею работать с нормалями к вершинам
            if (line.substr(0,2)=="v "){
                std::istringstream coordinates(line.substr(2));
                double firstCoordinates, secondCoordinates, thirdCoordinates;
                coordinates>>firstCoordinates>>secondCoordinates>>thirdCoordinates;
                cv::Vec3d lol(firstCoordinates, secondCoordinates, thirdCoordinates);
                model->addGlobalPoints(lol);
            }
            else if(line.substr(0,2)=="f ") {
                std::istringstream v(line.substr(1));
                std::string token;
                std::vector<int> kek;
                while(std::getline(v, token, ' '))
                {
                    v >> token;
                    std::istringstream tokenStream(token);
                    std::string a;
                    if (std::getline(tokenStream, a, '/'))
                    {
                        int n;
                        std::istringstream aStream(a);
                        aStream>>n;
                        kek.emplace_back(n - 1);
                    }
                }

                model->setIndexes(kek);
            }
        }
        infile.close();
        return model;
    }

    void objParser::write(const std::vector<std::shared_ptr<models::models>>& models, const std::string& pathToFile) {
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