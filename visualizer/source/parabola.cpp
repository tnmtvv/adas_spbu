#include "../include/parabola.h"

namespace models {
    // пользователю будет предоставлено либо отметить 3 точки на плоскости, которые будет считаны и по ним
    // будет восстановлена парабола, либо ввести эти точки как координаты
    [[maybe_unused]] parabola::parabola(const cv::Point2i& firstPoint, const cv::Point2i& secondPoint, const cv::Point2i& thirdPoint) {

        // почему представление x = ay^2 + by + c а не y = ax^2 + bx + c будет объяснено ниже
        // ay_1^2 + by_1 + c = x1
        // ay_2^2 + by_2 + c = x2
        // ay_3^2 + by_3 + c = x3

        int delta[3][3] = {{firstPoint.y * firstPoint.y, firstPoint.y, 1},
                          {secondPoint.y * secondPoint.y, secondPoint.y, 1},
                          {thirdPoint.y * thirdPoint.y, thirdPoint.y, 1}};
        cv::Mat deltaMat(3, 3, CV_64FC1, delta);

        int deltaA[3][3] = {{firstPoint.x, firstPoint.y, 1},
                           {secondPoint.x, secondPoint.y, 1},
                           {thirdPoint.x, thirdPoint.y, 1}};
        cv::Mat deltaForFirstCoefficient(3, 3, CV_64FC1, deltaA);

        int deltaB[3][3] = {{firstPoint.y * firstPoint.y, firstPoint.x, 1},
                            {secondPoint.y * secondPoint.y, secondPoint.x, 1},
                            {thirdPoint.y * thirdPoint.y, thirdPoint.x, 1}};
        cv::Mat deltaForSecondCoefficient(3, 3, CV_64FC1, deltaB);

        int deltaC[3][3] = {{firstPoint.y * firstPoint.y, firstPoint.y, firstPoint.x},
                           {secondPoint.y * secondPoint.y, secondPoint.y, firstPoint.x},
                           {thirdPoint.y * thirdPoint.y, thirdPoint.y, firstPoint.x}};
        cv::Mat deltaForThirdCoefficient(3, 3, CV_64FC1, deltaC);

        auto denominator = cv::determinant(deltaMat);
        auto numeratorForFirstCoefficient = cv::determinant(deltaForFirstCoefficient);
        auto numeratorForSecondCoefficient = cv::determinant(deltaForSecondCoefficient);
        auto numeratorForThirdCoefficient = cv::determinant(deltaForThirdCoefficient);

        firstCoefficient  = numeratorForFirstCoefficient / denominator;
        secondCoefficient  = numeratorForSecondCoefficient / denominator;
        thirdCoefficient  = numeratorForThirdCoefficient / denominator;

        //-------------------Тут полный бред-----------------------------------
        // пока я прредставляю это так :
        /* Для создания границы в классе plane будет чисто вирутальная функция CreateBorder которая будет принимать точки
         * и стратегию (тип границ) и классы наследники plane (например planeOfParabolaBorder)
         * Для создания границы нужно будет отметить 3 точки где на плоскости, найти пересечения с границами плоскости
         * и по этим точкам построить параболу,
         * которая будет новой границей.
         * Старую границу плоскости(когда она еще была прямоугольником) просто не отображаем (точнее ее часть),
         * однако запоминаем ее (для этого есть поля width и length в классе plane).
         * Если надо будет изменить параболу на другую или поменять тип, то забываем уже про новую границу-параболу, и
         * возращаемся к старой границе, проделывая ту же операцию.
         * Надо не забыть при рисовании границ, сделать вид сверху, но поменять оси x и y местами,
         * т.к в локальной системе координат плоскости --- ось y вдоль плоскости, то и границы должны идти вдоль y,
         * т.е. уравнение параболы x = a*y^2 + by + c */

        /*
        this->localPoints.emplace_back(offset, 0 , 0);

        double step = 0.5;

        for (auto counter = 0; counter < numberOfVertex; counter++) {
            double x = offset + step;
            double y = sqrt(2 * parameter * (x - offset));
            this->localPoints.emplace_back(x + step, y,  0);
            this->localPoints.emplace_back(x - step, -y, 0);
            step += step;
        }

        indexes.emplace_back(std::vector<int>{0, 1});
        indexes.emplace_back(std::vector<int>{0, 2});

        for (auto counter = 3; counter < numberOfVertex; counter++) {
            if (counter % 2 == 0) {
                indexes[1].emplace_back(counter);
            }
            else {
                indexes[0].emplace_back(counter);
            }
        }

        for (auto & localPoint : localPoints)
        {
            globalPoints.emplace_back(coordinateSystem->moveToGlobalCoordinates(localPoint));
        }*/
    }

    [[maybe_unused]] cv::Vec3d parabola::parabolaCoefficient() const {
        return {firstCoefficient, secondCoefficient, thirdCoefficient};
    }
}
