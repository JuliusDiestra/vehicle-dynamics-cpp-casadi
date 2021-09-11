cmake_minimum_required(VERSION 3.10.2)

project(marsvin_example LANGUAGES CXX)

# Dirty trick to use add_subdirectory for a directory that is not a subdirectory
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../marsvin ${CMAKE_CURRENT_BINARY_DIR}/marsvin )

add_executable(${PROJECT_NAME} 
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

target_link_libraries(${PROJECT_NAME}
    libmarsvin
)

