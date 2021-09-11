cmake_minimum_required(VERSION 3.10.2)

project(libmarsvin LANGUAGES CXX)

################
# CASADI set-up
################
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
# add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=0)
set(CASADI_PATH 
    ${CMAKE_CURRENT_SOURCE_DIR}/../casadi
)

file(GLOB CASADI_SOURCES ${CASADI_PATH} "casadi/core/*.cpp" )

##################
# MARSVIN
#################
set(MARSVIN_TOOLS_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/tools/src/marsvin_tools.cpp
)

set(MARSVIN_STATE_SPACE_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/state-space/car/src/car_nonlinear.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/state-space/car/src/car_linear.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/state-space/car/src/car_nonlinear_vx.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/state-space/car/src/car_linear_vx.cpp
)

# Adding source files
add_library(${PROJECT_NAME}
    SHARED
        ${CASADI_SOURCES}
        ${MARSVIN_STATE_SPACE_SRC}            
        ${MARSVIN_TOOLS_SRC}            
)

# Adding include directories
target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CASADI_PATH}/include
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/tools/include
        ${CMAKE_CURRENT_SOURCE_DIR}/state-space/car/include
)

target_link_libraries(${PROJECT_NAME}
    ${CASADI_PATH}/libcasadi.so
)

