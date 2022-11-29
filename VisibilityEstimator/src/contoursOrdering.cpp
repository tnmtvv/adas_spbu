#include "contoursOrdering.h"

using namespace std;
using namespace cv;

namespace EdgeDetector
{
    vector<int> contoursOrdering::sortContoursByTheirSizesDescending(vector<vector<Point>> contours)
    {
        vector<pair<int, int>> sizeOfContours;
        vector<int> sortedContoursIndices;

        for (int contourIndex = 0; contourIndex < contours.size(); contourIndex++)
        {
            sizeOfContours.push_back({ contourIndex, contours[contourIndex].size() });
        }

        sort(sizeOfContours.begin(), sizeOfContours.end(),
            [](auto& left, auto& right)
            {
                return left.second > right.second;
            });

        for (int i = 0; i < sizeOfContours.size(); i++)
        {
            sortedContoursIndices.push_back(sizeOfContours[i].first);
        }

        return sortedContoursIndices;
    }

    Point contoursOrdering::findPointWithMaxY(vector<Point> contour)
    {
        return *max_element(contour.begin(), contour.end(),
            [](auto& left, auto& right)
            {
                return left.y > right.y;
            });
    }
}