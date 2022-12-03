#ifndef VISUALIZER_BORDERS_H
#define VISUALIZER_BORDERS_H

#include "models.h"
#include "BorderValidator.h"

namespace models {

    class Borders: public models{
    protected:
        std::shared_ptr<BorderValidator> validator;
    public:
        virtual double findBorderValue(double y) = 0;

        [[maybe_unused]] bool isValid();
        virtual bool specificBorderIsValid() = 0;
    };

}

#endif
