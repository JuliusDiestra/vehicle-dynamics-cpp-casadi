#ifndef CAR_NONLINEAR_VX_STATE_SPACE_HPP_
#define CAR_NONLINEAR_VX_STATE_SPACE_HPP_

#include "casadi/casadi.hpp"
#include "mt_tools.hpp"
#include "car_nonlinear.hpp"

class CarNonlinearVxStateSpace : public CarNonlinearStateSpace {
    public:
        CarNonlinearVxStateSpace();
        void CalculateExternalForces() override;
        void CalculateGeneralizedForces() override;
        void DefineStateSpace() override;
};

#endif // CAR_NONLINEAR_VX_STATE_SPACE_HPP_
