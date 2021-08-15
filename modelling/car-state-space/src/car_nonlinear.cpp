#include "car_nonlinear.hpp"

// Bob Constructor
CarNonlinearStateSpace::CarNonlinearStateSpace(){};

// Run everything
void CarNonlinearStateSpace::Run() {
    CreateSymbolics();
    CreateRotationMatrices();
    CreatePositionVectors();
    CalculateVelocityVectors();
    CalculateKineticEnergy();
    CalculateExternalForces();
    CalculateGeneralizedForces();
    CalculateDynamics();
    DefineStateSpace();
}

// Create Symbolcis used for modelling
void CarNonlinearStateSpace::CreateSymbolics() {
    // States
    x = casadi::MX::sym("x");
    y = casadi::MX::sym("y"); 
    psi = casadi::MX::sym("psi");
    // States derivatives
    d_x = casadi::MX::sym("d_x");
    d_y = casadi::MX::sym("d_y");
    d_psi = casadi::MX::sym("d_psi");
    // Inputs
    delta = casadi::MX::sym("delta");
    Fxw2 = casadi::MX::sym("Fxw2");
    // Longitudinal velocity
    vx = casadi::MX::sym("vx");
    // Lateral velocity
    vy = casadi::MX::sym("vy");
    // Parameters
    l1 = casadi::MX::sym("l1");
    l2 = casadi::MX::sym("l2");
    Cy1 = casadi::MX::sym("Cy1");
    Cy2 = casadi::MX::sym("Cy2");
    m = casadi::MX::sym("m");
    J = casadi::MX::sym("J");

}

// Create Rotation Matrices used for modelling
void CarNonlinearStateSpace::CreateRotationMatrices() {
    R_EV = mt.Rz(psi);
    R_EW = mt.Rz(psi+delta);  
}

// Create Vectors
void CarNonlinearStateSpace::CreatePositionVectors() {
    // Generalized Coordinates
    q = casadi::MX::vertcat({x,y,psi});
    d_q = casadi::MX::vertcat({d_x,d_y,d_psi});
    // Position vectors 
    r_V1 = casadi::MX::vertcat({l1,0});
    r_V2 = casadi::MX::vertcat({-l2,0});
    r = casadi::MX::vertcat({x,y});
    r1 = r + casadi::MX::mtimes(R_EV,r_V1);
    r2 = r + casadi::MX::mtimes(R_EV,r_V2);
}

void CarNonlinearStateSpace::CalculateVelocityVectors() {
    d_r = mt.ChainRule(r,q,d_q);
    v1 = mt.ChainRule(r1,q,d_q);
    v2 = mt.ChainRule(r2,q,d_q);
    vw1 = casadi::MX::mtimes(R_EW.T(),v1);
    vw2 = casadi::MX::mtimes(R_EV.T(),v2);
    vel = casadi::MX::vertcat({vx,vy});
}

void CarNonlinearStateSpace::CalculateKineticEnergy() {
    T = 0.5*m*casadi::MX::mtimes(d_r.T(),d_r) +0.5*J*d_psi*2;
}

void CarNonlinearStateSpace::CalculateExternalForces() {
    Sy1 = vw1(1)/vw1(0);
    Sy2 = vw2(1)/vw2(0);
    Fxw1 = 0;
    Fyw1 = - Cy1*Sy1;
    Fyw2 = - Cy2*Sy2;
    F1 = casadi::MX::mtimes(R_EW,casadi::MX::vertcat({Fxw1,Fyw1}));
    F2 = casadi::MX::mtimes(R_EV,casadi::MX::vertcat({Fxw2,Fyw2}));
    F = 0;
}

void CarNonlinearStateSpace::CalculateGeneralizedForces() {
    casadi::MX Q_temp = casadi::MX::mtimes(F1.T(),casadi::MX::jacobian(r1,q)) + casadi::MX::mtimes(F2.T(),casadi::MX::jacobian(r2,q)); 
    Q = Q_temp.T();
}

void CarNonlinearStateSpace::CalculateDynamics() {
    T_q = casadi::MX::jacobian(T,q);
    T_dq = casadi::MX::jacobian(T,d_q);
    // Review following line
    //dd_q = casadi::MX::mtimes(casadi::MX::jacobian(T_dq,d_q),-casadi::MX::mtimes(casadi::MX::jacobian(T_dq,q),d_q)+T_q+Q);
    dd_q = casadi::MX::mtimes(casadi::MX::inv(casadi::MX::jacobian(T_dq.T(),d_q)),-casadi::MX::mtimes(casadi::MX::jacobian(T_dq.T(),q),d_q)+T_q.T()+Q);
    dd_q_v = casadi::MX::substitute(dd_q,casadi::MX::vertcat({d_x,d_y}),casadi::MX::vertcat({vx*casadi::MX::cos(psi)-vy*casadi::MX::sin(psi),vy*casadi::MX::cos(psi)+vx*casadi::MX::sin(psi)}));
    dotR_EV = casadi::MX::vertcat({
        casadi::MX::horzcat({-casadi::MX::sin(psi)*d_psi,-casadi::MX::cos(psi)*d_psi}),
        casadi::MX::horzcat({casadi::MX::cos(psi)*d_psi,-casadi::MX::sin(psi)*d_psi})
        });
    casadi::MX temp;
    // Change expression from dd_x and dd_y to d_vx and d_vy:
    d_vel = casadi::MX::mtimes(R_EV.T(),casadi::MX::vertcat({dd_q_v(0),dd_q_v(1)})) - casadi::MX::mtimes(dotR_EV,vel);
    //dd_q_v = casadi::MX::vertcat({d_vel(0),d_vel(1),dd_q_v(2)});
    dotX = casadi::MX::vertcat({d_vel(0),d_vel(1),dd_q_v(2)});
    dd_q_frame_v = casadi::MX::vertcat({d_vel(0),d_vel(1),dd_q_v(2)});
}

void CarNonlinearStateSpace::DefineStateSpace() {
    // State-space: Nonlinear equation
    /*
        dotX = f(X,U)
    */
    // State
    X =  casadi::MX::vertcat({vx,vy,psi});
    // Inputs
    U = casadi::MX::vertcat({delta,Fxw2});
    // State function
    dotX = dd_q_frame_v; 
}

casadi::MX CarNonlinearStateSpace::GetStateVector() {
    return X;
}

casadi::MX CarNonlinearStateSpace::GetInputVector() {
    return U;
}

casadi::MX CarNonlinearStateSpace::GetFunctionVector() {
    return dotX;
}

