
#ifndef CAR_LINEAR_VX_STATE_SPACE_HPP_
#define CAR_LINEAR_VX_STATE_SPACE_HPP_

#include "casadi/casadi.hpp"
#include "mt_tools.hpp"
#include "car_nonlinear_vx.hpp"

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

#endif // CAR_LINEAR_VX_STATE_SPACE_HPP_
