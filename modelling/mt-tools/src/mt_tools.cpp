
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

