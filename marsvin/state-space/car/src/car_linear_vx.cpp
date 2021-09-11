#include "marsvin_car_linear_vx.hpp"

// Constructor
marsvin::CarLinearVxStateSpace::CarLinearVxStateSpace(){};

void marsvin::CarLinearVxStateSpace::Linearize() {
    A = casadi::MX::substitute(casadi::MX::jacobian(dotX,X),casadi::MX::vertcat({X,U}),casadi::MX::vertcat({0,0,0}));
    B = casadi::MX::substitute(casadi::MX::jacobian(dotX,U),casadi::MX::vertcat({X,U}),casadi::MX::vertcat({0,0,0}));
    dotX = casadi::MX::mtimes(A,X) + casadi::MX::mtimes(B,U);
};

casadi::MX marsvin::CarLinearVxStateSpace::GetA() {
   return A; 
};

casadi::MX marsvin::CarLinearVxStateSpace::GetB() {
    return B;
};

