#include "car_nonlinear_vx.hpp"

// Constructor
CarNonlinearVxStateSpace::CarNonlinearVxStateSpace(){};

void CarNonlinearVxStateSpace::CalculateExternalForces() {
    Sy1 = vw1(1)/vw1(0);
    Sy2 = vw2(1)/vw2(0);
    Fxw1 = 0;
    Fxw2 = 0;
    Fyw1 = - Cy1*Sy1;
    Fyw2 = - Cy2*Sy2;
    F1 = casadi::MX::mtimes(R_EV,casadi::MX::vertcat({Fxw1,Fyw1*casadi::MX::cos(delta)}));
    F2 = casadi::MX::mtimes(R_EV,casadi::MX::vertcat({Fxw2,Fyw2}));
};

void CarNonlinearVxStateSpace::CalculateGeneralizedForces() {
    casadi::MX Q_temp = casadi::MX::mtimes(F1.T(),casadi::MX::jacobian(r1,q)) + casadi::MX::mtimes(F2.T(),casadi::MX::jacobian(r2,q));
    Q = Q_temp.T();
};

void CarNonlinearVxStateSpace::DefineStateSpace() {
    // State-space: Nonlinear equation
    /*
        dotX = f(X,U)
    */
    // State
    X =  casadi::MX::vertcat({vy,psi});
    // Inputs
    U = casadi::MX::vertcat({delta});
    // State function
    dotX = casadi::MX::vertcat({dd_q_frame_v(1),dd_q_frame_v(2)});
};

