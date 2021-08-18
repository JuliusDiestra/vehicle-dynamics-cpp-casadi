#include "car_linear_vx.hpp"

// Constructor
CarLinearVxStateSpace::CarLinearVxStateSpace(){};

void CarLinearVxStateSpace::Linearize() {
    A = casadi::MX::substitute(casadi::MX::jacobian(dotX,X),casadi::MX::vertcat({X,U}),casadi::MX::vertcat({0,0,0}));
    B = casadi::MX::substitute(casadi::MX::jacobian(dotX,U),casadi::MX::vertcat({X,U}),casadi::MX::vertcat({0,0,0}));
    dotX = casadi::MX::mtimes(A,X) + casadi::MX::mtimes(B,U);
};

casadi::MX CarLinearVxStateSpace::GetA() {
   return A; 
};

casadi::MX CarLinearVxStateSpace::GetB() {
    return B;
};
