#include "../include/Road.h"

namespace models {

    [[maybe_unused]] std::vector<std::vector<cv::Vec3d>> Road::getRoadPoints(double stepOfYAxis) {
        std::vector<std::vector<cv::Vec3d>> points;
        for (auto & plane : planes) {
            auto rightBorderModels = plane->getRightBorderModels();
            auto leftBorderModels = plane->getLeftBorderModels();

            for (int p = 0; p < leftBorderModels.size(); p++) {
                auto localPointsForLeftBorderModel = leftBorderModels[p]->getLocalPoints();
                auto localPointsForRightBorderModel = rightBorderModels[p]->getLocalPoints();
                auto currentY = localPointsForLeftBorderModel[0].y;
                auto maximumY = localPointsForLeftBorderModel[1].y;
                while (currentY < maximumY) {
                    auto firstX = leftBorderModels[p]->findBorderValue(currentY);
                    auto secondX = rightBorderModels[p]->findBorderValue(currentY);
                    auto firstPointInGlobalCoordinates = plane->getCoordinateSystem()->moveToGlobalCoordinates(cv::Vec3d(firstX, currentY, 0));
                    auto secondPointInGlobalCoordinates = plane->getCoordinateSystem()->moveToGlobalCoordinates(cv::Vec3d(secondX, currentY, 0));
                    std::vector<cv::Vec3d> constYPoints = {firstPointInGlobalCoordinates, secondPointInGlobalCoordinates};
                    points.emplace_back(constYPoints);
                    currentY += stepOfYAxis;
                }

                //----------Копипаст
                auto firstX = leftBorderModels[p]->findBorderValue(maximumY);
                auto secondX = rightBorderModels[p]->findBorderValue(maximumY);
                auto firstPointInGlobalCoordinates = plane->getCoordinateSystem()->moveToGlobalCoordinates(cv::Vec3d(firstX, currentY, 0));
                auto secondPointInGlobalCoordinates = plane->getCoordinateSystem()->moveToGlobalCoordinates(cv::Vec3d(secondX, currentY, 0));
                std::vector<cv::Vec3d> constYPoints = {firstPointInGlobalCoordinates, secondPointInGlobalCoordinates};
                points.emplace_back(constYPoints);
                //----------Конец копипаста
            }
        }

        return points;
    }

    [[maybe_unused]] [[maybe_unused]] void Road::addBorders(const std::shared_ptr<plane>& p, std::vector<cv::Point2d> vector) {
        auto models = strategy->Strategy(std::move(vector));

        if (models->getGlobalPoints()[0].y > models->getGlobalPoints()[models->getGlobalPoints().size() - 1].y) {
            std::reverse(models->getGlobalPoints().begin(), models->getGlobalPoints().end());
        }
        if (models->getGlobalPoints().back().x < 0) {
            //leftBorder.insert(leftBorder.end(), models->getGlobalPoints().begin(), models->getGlobalPoints().end());
            p->addLeftBorder(models);
        }
        else {
            //rightBorder.insert(rightBorder.end(), models->getGlobalPoints().begin(), models->getGlobalPoints().end());
            p->addRightBorder(models);
        }
    }

    [[maybe_unused]] [[maybe_unused]] void Road::setStrategy(std::shared_ptr<StrategyOfAddBorders> newStrategy) {
        strategy = std::move(newStrategy);
    }

    [[maybe_unused]] void Road::IsValidBorders(){

    }
}