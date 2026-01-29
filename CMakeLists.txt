cmake_minimum_required(VERSION 3.12)
project(ekf_pybind)

find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)

set(KEISAN_INCLUDE "/home/salsa-hanaa/ichiro-ws/src/keisan/include")

add_library(ekf_lib STATIC 
    /home/salsa-hanaa/ichiro-ws/src/keisan/src/ekf/ekf2_ball.cpp
)

target_include_directories(ekf_lib PUBLIC ${KEISAN_INCLUDE} ${EIGEN3_INCLUDE_DIR})

# Buat modul Python
pybind11_add_module(ekf_module ekf_binding.cpp)

# Link modul ke library tadi
target_link_libraries(ekf_module PRIVATE ekf_lib)

# Ekf_module tahu path include-nya
target_include_directories(ekf_module PRIVATE ${KEISAN_INCLUDE})