#ifndef MT_TOOLS_H_
#define MT_TOOLS_H_

#include "casadi/casadi.hpp"

namespace mt {

class tools {
    public:
        tools();
        casadi::MX Rz(const casadi::MX& angle);
        casadi::MX ChainRule(const casadi::MX& f, const casadi::MX& z, const casadi::MX& d_z); 
};

}
#endif // MT_TOOLS_H_
