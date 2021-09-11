#ifndef MARSVIN_STATE_SPACE_CAR_NONLINEAR_VX_HPP_
#define MARSVIN_STATE_SPACE_CAR_NONLINEAR_VX_HPP_

#include "marsvin_car_nonlinear.hpp"

namespace marsvin {

class CarNonlinearVxStateSpace : public CarNonlinearStateSpace {
    public:
        CarNonlinearVxStateSpace();
        void CalculateExternalForces() override;
        void CalculateGeneralizedForces() override;
        void DefineStateSpace() override;
};
}

#endif // MARSVIN_STATE_SPACE_CAR_NONLINEAR_VX_HPP_

