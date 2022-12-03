#include "../include/Borders.h"

namespace models {
    [[maybe_unused]] bool Borders::isValid() {
        auto points = this->getLocalPoints();

        double firstX = points[0].x;

        for (int i = 1 ; i < points.size(); i++) {
            if (points[i].x * firstX < 0) {
                return false;
            }
        }

        return true;
    }
} // models