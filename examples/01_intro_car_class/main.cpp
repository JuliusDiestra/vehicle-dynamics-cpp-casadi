#include "marsvin.hpp"

int main() {
    // Nonlinear state-space
    marsvin::CarNonlinearStateSpace carNonlinear;
    carNonlinear.Run();
    casadi::MX U = carNonlinear.GetInputVector();
    casadi::MX X = carNonlinear.GetStateVector();
    casadi::MX parameters = carNonlinear.GetParameters();
    std::cout << "Input: " << U << std::endl;
    std::cout << "State: " << X << std::endl;
    std::cout << "Parameters: " << parameters << std::endl;
}

