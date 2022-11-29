#include "contoursMerger.h"

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "contoursOrdering.h"

#include <filesystem>
#include <algorithm>
#include <map>
#include <set>

namespace EdgeDetector
{
    std::vector<std::vector<cv::Point>> contoursMerger::connectContours(std::vector<std::vector<cv::Point>> contours,
        int imageSizeX, int imageSizeY)
    {
        // Making matrix where points are -1 if they don't belong to any contour, and they can be positive,
        // if they belong to any contour; and positive value is the index of the contour they belong to.
        std::vector<std::vector<int>> indexMatrix(imageSizeX, std::vector(imageSizeY, -1));
        for (int contour = 0; contour < contours.size(); contour++)
        {
            for (int pointIndex = 0; pointIndex < contours[contour].size(); pointIndex++)
            {
                indexMatrix[contours[contour][pointIndex].x][contours[contour][pointIndex].y] = contour;
            }
        }

        // Sorting by values the dictionary where key is contour index, value is contour size.
        auto sortedContoursIndices = contoursOrdering::sortContoursByTheirSizesDescending(contours);

        // подумать, чем ограничить количество итераций
        // Merging contours.
        std::vector<std::vector<cv::Point>> newContours(contours);
        for (int i = 0; i < (sortedContoursIndices.size() < 15 ? sortedContoursIndices.size() : 15); i++)
        {
            newContours = mergeContours(newContours, indexMatrix, sortedContoursIndices[i], contours[sortedContoursIndices[i]]);
        }

        return removeZeroSizeContours(newContours);
    }

    std::set<int> contoursMerger::findNearestContours(std::vector<std::vector<cv::Point>> contours, int mainContourIndex,
        std::vector<std::vector<int>> contoursIndexesMatrix, cv::Point point)
    {
        std::set<int> foundContours;
        const int neighborhoodSize = 5;

        for (int xDiff = -neighborhoodSize; xDiff <= neighborhoodSize; xDiff++)
        {
            for (int yDiff = -neighborhoodSize; yDiff <= neighborhoodSize; yDiff++)
            {
                if (point.x + xDiff <= 0 || point.y + yDiff <= 0
                    || point.x + xDiff >= contoursIndexesMatrix.size() || point.y + yDiff >= contoursIndexesMatrix[0].size())
                {
                    continue;
                }
                auto contourIndex = contoursIndexesMatrix[point.x + xDiff][point.y + yDiff];
                if (contourIndex != -1 && contours[contourIndex].size() != 0 && contourIndex != mainContourIndex)
                {
                    foundContours.insert(contourIndex);
                }
            }
        }

        return foundContours;
    }

    /// <summary>
    /// Returns vector where some contours have been merged.
    /// </summary>
    /// <param name="contours"></param>
    /// <param name="mainContourIndex"></param>
    /// <param name="nearestContours"></param>
    /// <returns></returns>
    std::vector<std::vector<cv::Point>> contoursMerger::mergeContours(std::vector<std::vector<cv::Point>> contours,
        std::vector<std::vector<int>> indexMatrix,
        int mainContourIndex, std::vector<cv::Point> mainContour)
    {
        auto pointWithMaxY = contoursOrdering::findPointWithMaxY(mainContour);

        auto nearestContours = findNearestContours(contours, mainContourIndex, indexMatrix, pointWithMaxY);

        if (nearestContours.size() == 0)
        {
            return contours;
        }

        std::vector<cv::Point> zeroSizeVector;
        std::vector<std::vector<cv::Point>> mergedContours;
        mergedContours.reserve(contours.size() - nearestContours.size());
        for (int i = 0; i < contours.size(); i++)
        {
            if (i == mainContourIndex)
            {
                int sizeOfMergedContour = contours[mainContourIndex].size();
                for (auto contourIndex : nearestContours)
                {
                    sizeOfMergedContour += contours[contourIndex].size();
                }

                std::vector<cv::Point> newContour;
                newContour.reserve(sizeOfMergedContour);
                newContour.insert(newContour.end(), contours[mainContourIndex].begin(), contours[mainContourIndex].end());
                for (auto neighbor : nearestContours)
                {
                    newContour.insert(newContour.end(), contours[neighbor].begin(), contours[neighbor].end());
                }

                mergedContours.push_back(newContour);

                continue;
            }

            if (nearestContours.find(i) != nearestContours.end())
            {
                mergedContours.push_back(zeroSizeVector);
                continue;
            }

            mergedContours.push_back(contours[i]);
        }

        return mergeContours(mergedContours, indexMatrix, mainContourIndex, mergedContours[mainContourIndex]);
    }

    std::vector<std::vector<cv::Point>> contoursMerger::removeZeroSizeContours(std::vector<std::vector<cv::Point>> contours)
    {
        std::vector<std::vector<cv::Point>>::iterator iter = contours.begin();
        while (iter != contours.end())
        {
            if (iter->size() == 0)
            {
                iter->clear();
                iter = contours.erase(iter);
                continue;
            }

            advance(iter, 1);
        }

        return contours;
    }
}