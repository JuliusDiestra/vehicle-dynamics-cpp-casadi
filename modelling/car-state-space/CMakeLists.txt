cmake_minimum_required(VERSION 3.10.2)

project(car_state_space LANGUAGES CXX)
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
# add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=0)
set(CASADI_PATH 
    ${CMAKE_CURRENT_SOURCE_DIR}/../../casadi
)

file(GLOB CASADI_SOURCES ${CASADI_PATH} "casadi/core/*.cpp" )

add_executable(${PROJECT_NAME} 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/car_nonlinear.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/car_nonlinear_vx.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/car_linear.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/car_linear_vx.cpp
    ${CASADI_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/../mt-tools/src/mt_tools.cpp
)

# Add include directory
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CASADI_PATH}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../mt-tools/include
)

target_link_libraries(${PROJECT_NAME}
    ${CASADI_PATH}/libcasadi.so
)

