# cmake_minimum_required(VERSION 3.12)
# project(ekf_pybind)

# find_package(pybind11 REQUIRED)
# find_package(Eigen3 REQUIRED)

# # Path absolut ke header dan file keisan
# include_directories(/home/salsa-hanaa/ichiro-ws/src/keisan/include)  # Untuk ekf2_ball.hpp
# include_directories(${EIGEN3_INCLUDE_DIR})

# # Compile modul dengan path absolut
# pybind11_add_module(ekf_module 
#     ekf_binding.cpp 
#     /home/salsa-hanaa/ichiro-ws/src/keisan/src/ekf/ekf2_ball.cpp
# )

cmake_minimum_required(VERSION 3.12)
project(ekf_pybind)

find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(/home/salsa-hanaa/ichiro-ws/src/keisan/include)
include_directories(${EIGEN3_INCLUDE_DIR})

# Compile ekf2_ball.cpp sebagai library statis
add_library(ekf_lib STATIC /home/salsa-hanaa/ichiro-ws/src/keisan/src/ekf/ekf2_ball.cpp)

# Compile binding saja, link ke library (gunakan PRIVATE untuk keyword signature)
pybind11_add_module(ekf_module ekf_binding.cpp)
target_link_libraries(ekf_module PRIVATE ekf_lib)