#include "marsvin.hpp"

int main() {
    // Nonlinear state-space
    marsvin::CarLinearVxStateSpace car;
    car.Run();
    casadi::MX U = car.GetInputVector();
    casadi::MX X = car.GetStateVector();
    casadi::MX parameters = carNonlinear.GetParameters();
    std::cout << "Input: " << U << std::endl;
    std::cout << "State: " << X << std::endl;
    std::cout << "Parameters: " << parameters << std::endl;
    /*
        TO BE CONTINUED ...
    */
}

