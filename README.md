# Vehicle dynamics using C++: State-space models
------------------------------------------------------------------------------

This repository contains the **MARSVIN** library that contains linear and nonlinear state-space models from vehicles 
and some useful functions for modelling.

The library depends a lot on **CasADi**, therefore you need to install CasADI previously or download their binaries.
Some **CasADi** classes are used in some **MARSVIN** classes.

CasADi Official Page: [https://web.casadi.org/](https://web.casadi.org/)

CasADi Github : [https://github.com/casadi/casadi](https://github.com/casadi/casadi)

Current implemented classes:

- **marsvin::CarNonlinearStateSpace :** Nonlinear car state-space model. Modelling included in the class. 
    -  **Inputs:** Steering and Braking.
    -  **States:** Longitudinal velocity, lateral velocity and yaw angle.
- **marsvin::CarLinearStateSpace :** Linear car state-space model. Modelling included in the class. 
    - **Inputs:** Steering and Braking. 
    - **States:** Longitudinal velocity, lateral velocity and yaw angle.
- **marsvin::CarNonlinearVxStateSpace :** Nonlinear car state-space model assuming constant longitudinal velocity. Modelling included in the class. 
    - **Inputs:** Steering.
    - **States:** Lateral velocity and yaw angle.
- **marsvin::CarLinearVxStateSpace :** Linear car state-space model assuming constant longitudinal velocity. Modelling included in the class. 
    - **Inputs:** Steering. 
    - **States:** Lateral velocity and yaw angle.

Addintianlly, one extra class with useful functions:
- **marsvin::tools :** This class contatins some functions useful for modelling.
    - **marsvin::toools::Rz :** Calculation of rotation matrix around axis Z. 
    - **marsvin::toools::ChainRule :** Chain rule to calculate df/dt while 'f' depends on 'x', and 'x' depends on 't'. 
    - **marsvin::toools::rk4 :** Next discrete step calculation using Runge-Kutta 4 for state-stace equations. 

## How to use MARSVIN library?

### 1) First, install CasADi
I do not trust myself, so I did not build **CasADi** from source code. I downloaded the already built binaries.
I follow the following steps:
1. Download binaries from Oficial CasADi page [here](https://web.casadi.org/get/). 
My OS is Linux, also I used Matlab while I get lazy, so I downloaded Matlab binaries for Linux.
2. Unpack the file you downloaded, i.e. **casadi-linux-matlabR2014b-v3.5.5.tar.gz** file. Note: The version might change.
3. Create a directory in **vehicle-dynamics-cpp-casadi** called **casadi**. Note: This directory is included in .gitignore
4. Copy the unpacked files from **Point 2** into the directory **casadi**.

### 2) Run our example
Do this:
1. Go to **vehicle-dynamics-cpp-casadi/examples/01_intro_car_class**
2. Run our bash script to run the example. Run this in your Terminal:
```
bash build-code.sh
```

# Some notes
* The scrips are located in **marsvin** directory
* The directory **marsvin** has a CMakeLists.txt that already builds the **libmarsvin** library.
* The **libmarsvin** already linked the CasADi libraries.
* In order to use **libmarsvin** you might need to link it to your target and include **marsvin.hpp** in your code.

