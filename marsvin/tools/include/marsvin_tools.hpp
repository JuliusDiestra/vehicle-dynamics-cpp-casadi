#ifndef MARSVIN_TOOLS_HPP__
#define MARSVIN_TOOLS_HPP__

#include "casadi/casadi.hpp"

namespace marsvin {

class tools {
    public:
        tools();
        casadi::MX Rz(const casadi::MX& angle);
        casadi::MX ChainRule(const casadi::MX& f, const casadi::MX& z, const casadi::MX& d_z);
        casadi::MX rk4(const casadi::MX& f, const casadi::MX& X, const casadi::MX& U, const casadi::MX& Ts);
};
}

#endif // MARSVIN_TOOLS_HPP__

