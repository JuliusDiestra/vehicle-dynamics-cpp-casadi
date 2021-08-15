#include "car_nonlinear.hpp"

int main() {
    CarNonlinearStateSpace car;
    car.Run(); 
    casadi::MX U = car.GetInputVector();
    casadi::MX X = car.GetStateVector();
    std::cout << "Input: " << U << std::endl;
    std::cout << "State: " << X << std::endl;
}

