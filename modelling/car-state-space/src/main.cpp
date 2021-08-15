#include "car_nonlinear.hpp"
#include "car_nonlinear_vx.hpp"

int main() {
    // Nonlinear state-space
    CarNonlinearStateSpace car;
    car.Run(); 
    casadi::MX U = car.GetInputVector();
    casadi::MX X = car.GetStateVector();
    std::cout << "###### Nonlinear #######" << std::endl;
    std::cout << "Input: " << U << std::endl;
    std::cout << "State: " << X << std::endl;
    // Nonlinear state-space : Constant velocity
    CarNonlinearVxStateSpace carVx;
    carVx.Run(); 
    casadi::MX U_vx = carVx.GetInputVector();
    casadi::MX X_vx = carVx.GetStateVector();
    std::cout << "##### Nonlinear Constant Velocity ######" << std::endl;
    std::cout << "Input: " << U_vx << std::endl;
    std::cout << "State: " << X_vx << std::endl;
}

