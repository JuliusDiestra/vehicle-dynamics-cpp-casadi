
#include "mt_tools.hpp"

mt::tools::tools(){};

casadi::MX mt::tools::Rz(const casadi::MX& angle) {
    return casadi::MX::vertcat({
        casadi::MX::horzcat({casadi::MX::cos(angle),-casadi::MX::sin(angle)}),
        casadi::MX::horzcat({casadi::MX::sin(angle),casadi::MX::cos(angle)})
        });   
}

casadi::MX mt::tools::ChainRule(const casadi::MX& f, const casadi::MX& z, const casadi::MX& d_z) {
    return casadi::MX::mtimes(casadi::MX::jacobian(f,z),d_z);
}

casadi::MX mt::tools::rk4(const casadi::MX& f, const casadi::MX& X, const casadi::MX& U, const casadi::MX& Ts) {
    casadi::MX k1 = f;
    casadi::MX k2 = casadi::MX::substitute(f,casadi::MX::vertcat({X,U}),casadi::MX::vertcat({X+Ts*k1/2,U}));
    casadi::MX k3 = casadi::MX::substitute(f,casadi::MX::vertcat({X,U}),casadi::MX::vertcat({X+Ts*k2/2,U}));
    casadi::MX k4 = casadi::MX::substitute(f,casadi::MX::vertcat({X,U}),casadi::MX::vertcat({X+Ts*k3,U}));
    casadi::MX X_next = X + Ts*(k1 + 2*k2 + 2*k3 + k4)/6;
    return X_next;
}

