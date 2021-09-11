#ifndef MARSVIN_STATE_SPACE_CAR_LINEAR_VX_HPP_
#define MARSVIN_STATE_SPACE_CAR_LINEAR_VX_HPP_

#include "marsvin_car_nonlinear_vx.hpp"

namespace marsvin {

class CarLinearVxStateSpace : public CarNonlinearVxStateSpace {
    public:
        CarLinearVxStateSpace();
        void Linearize() override;
        casadi::MX GetA();
        casadi::MX GetB();
    protected:
        casadi::MX A;
        casadi::MX B;
};
}

#endif // MARSVIN_STATE_SPACE_CAR_LINEAR_VX_HPP_

