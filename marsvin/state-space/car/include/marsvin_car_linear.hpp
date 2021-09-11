#ifndef MARSVIN_STATE_SPACE_CAR_LINEAR_HPP_
#define MARSVIN_STATE_SPACE_CAR_LINEAR_HPP_

#include "marsvin_car_nonlinear.hpp"

namespace marsvin {

class CarLinearStateSpace : public CarNonlinearStateSpace {
    public:
        CarLinearStateSpace();
        void Linearize() override;
        casadi::MX GetA();
        casadi::MX GetB();
    protected:
        casadi::MX A;
        casadi::MX B;
};
}

#endif // MARSVIN_STATE_SPACE_CAR_LINEAR_HPP_

