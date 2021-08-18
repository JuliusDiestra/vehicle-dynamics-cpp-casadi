#ifndef CAR_NONLINEAR_STATE_SPACE_HPP_
#define CAR_NONLINEAR_STATE_SPACE_HPP_

#include "casadi/casadi.hpp"
#include "mt_tools.hpp"

class CarNonlinearStateSpace {
    public:
        CarNonlinearStateSpace();
        void Run();
        void CreateSymbolics();
        void CreateRotationMatrices();
        void CreatePositionVectors();
        void CalculateVelocityVectors();
        void CalculateKineticEnergy();
        virtual void CalculateExternalForces();
        virtual void CalculateGeneralizedForces();
        virtual void CalculateDynamics();
        virtual void DefineStateSpace();
        virtual void Linearize();
        casadi::MX GetStateVector();
        casadi::MX GetInputVector();
        casadi::MX GetFunctionVector();
    protected:
        mt::tools mt;
        casadi::MX x;
        casadi::MX y;
        casadi::MX psi;
        casadi::MX d_x;
        casadi::MX d_y;
        casadi::MX d_psi;
        casadi::MX delta;
        casadi::MX Fxw2;
        casadi::MX vx;
        casadi::MX vy;
        casadi::MX l1;
        casadi::MX l2;
        casadi::MX Cy1;
        casadi::MX Cy2;
        casadi::MX m;
        casadi::MX J;
        casadi::MX R_EV;
        casadi::MX R_EW;
        casadi::MX q;
        casadi::MX d_q;
        casadi::MX dd_q;
        casadi::MX dd_q_v;
        casadi::MX dd_q_frame_v;
        casadi::MX r_V1;
        casadi::MX r_V2;
        casadi::MX r;
        casadi::MX r1;
        casadi::MX r2;
        casadi::MX d_r;
        casadi::MX v1;
        casadi::MX v2;
        casadi::MX vw1;
        casadi::MX vw2;
        casadi::MX vel;
        casadi::MX d_vel;
        casadi::MX T;
        casadi::MX Q;
        casadi::MX X;
        casadi::MX U;
        casadi::MX dotX;
        casadi::MX T_q;
        casadi::MX T_dq;
        casadi::MX Sy1;
        casadi::MX Sy2;
        casadi::MX Fxw1;
        casadi::MX Fyw1;
        casadi::MX Fyw2;
        casadi::MX F1;
        casadi::MX F2;
        casadi::MX F;
        casadi::MX dotR_EV;
};

#endif // CAR_NONLINEAR_STATE_SPACE_HPP_
