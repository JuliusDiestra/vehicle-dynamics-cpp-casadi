#ifndef CAR_LINEAR_STATE_SPACE_HPP_
#define CAR_LINEAR_STATE_SPACE_HPP_

#include "casadi/casadi.hpp"
#include "mt_tools.hpp"
#include "car_nonlinear.hpp"

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

#endif // CAR_LINEAR_STATE_SPACE_HPP_

