#include "marsvin_car_linear.hpp"

// Constructor
marsvin::CarLinearStateSpace::CarLinearStateSpace(){};

void marsvin::CarLinearStateSpace::Linearize() {
    A = casadi::MX::substitute(casadi::MX::jacobian(dotX,X),casadi::MX::vertcat({X,U}),casadi::MX::vertcat({vx,0,0,0,0}));
    B = casadi::MX::substitute(casadi::MX::jacobian(dotX,U),casadi::MX::vertcat({X,U}),casadi::MX::vertcat({vx,0,0,0,0}));
    dotX = casadi::MX::mtimes(A,X) + casadi::MX::mtimes(B,U);
};

casadi::MX marsvin::CarLinearStateSpace::GetA() {
   return A; 
};

casadi::MX marsvin::CarLinearStateSpace::GetB() {
    return B;
};

