# Vehicle dynamics using C++: State-space models
------------------------------------------------------------------------------

This repository contains the **marsvin** library that contains linear and nonlinear state-space models from vehicles 
and some useful functions for modelling.

The library depends a lot on CasADI, therefore you need to install CasADI previously or download their binaries.
Some CasADI classes are used in some **marsvin** classes.

Current implemented classes:

* **marsvin::CarNonlinearStateSpace :** Nonlinear car state-space model. Modelling included in the class. 
    Inputs: Steering and Braking.
    States: Longitudinal velocity, lateral velocity and yaw angle.
* **marsvin::CarLinearStateSpace :** Linear car state-space model. Modelling included in the class. 
    Inputs: Steering and Braking. 
    States: Longitudinal velocity, lateral velocity and yaw angle.
* **marsvin::CarNonlinearVxStateSpace :** Nonlinear car state-space model assuming constant longitudinal velocity. Modelling included in the class. 
    Inputs: Steering.
    States: Lateral velocity and yaw angle.
* **marsvin::CarLinearVxStateSpace :** Linear car state-space model assuming constant longitudinal velocity. Modelling included in the class. 
    Inputs: Steering. 
    States: Lateral velocity and yaw angle.

