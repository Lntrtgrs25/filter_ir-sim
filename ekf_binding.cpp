// #include <pybind11/pybind11.h>
// #include <pybind11/stl.h>  // Untuk std::vector
// #include <pybind11/eigen.h>  // Jika Matrix adalah Eigen (untuk binding langsung)
// #include "/home/salsa-hanaa/ichiro-ws/src/keisan/src/ekf/ekf2_ball.cpp"
// namespace py = pybind11;

// PYBIND11_MODULE(ekf_module, m) {
//     py::class_<keisan::ekf_ball>(m, "EKFBall")  // Bind kelas keisan::ekf_ball sebagai EKFBall di Python
//         .def(py::init<>())  // Constructor default
//         .def("init", &keisan::ekf_ball::init)  // init(x, y, v, theta)
//         .def("predict", &keisan::ekf_ball::predict)  // predict(dt)
//         .def("update", &keisan::ekf_ball::update)  // update(z) - z sebagai Eigen::Matrix<2,1> atau vector
//         .def("getPosition", &keisan::ekf_ball::getPosition)  // Return Matrix<2,1>
//         .def("getVelocity", &keisan::ekf_ball::getVelocity)  // Return Matrix<2,1>
//         .def("getstate", &keisan::ekf_ball::getstate)  // Return Matrix<4,1>
//         .def("getcov", &keisan::ekf_ball::getcov);  // Return Matrix<4,4>
// }

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>  // Untuk std::vector
#include <pybind11/eigen.h>
#include "/home/salsa-hanaa/ichiro-ws/src/keisan/include/keisan/ekf/ekf2_ball.hpp"

namespace py = pybind11;

PYBIND11_MODULE(ekf_module, m) {
    py::class_<keisan::ekf_ball>(m, "EKFBall")
        .def(py::init<>())
        .def("init", &keisan::ekf_ball::init)
        .def("predict", &keisan::ekf_ball::predict)
        .def("update", [](keisan::ekf_ball &self, const std::vector<double> &z_vec) {
            if (z_vec.size() != 2) throw std::runtime_error("z must have 2 elements");
            keisan::Matrix<2, 1> z;
            z[0][0] = z_vec[0];
            z[1][0] = z_vec[1];
            self.update(z);
        })  
        .def("getPosition", &keisan::ekf_ball::getPosition)
        .def("getVelocity", &keisan::ekf_ball::getVelocity);
        .def("getstate", &keisan::ekf_ball::getstate) 
        .def("getcov", &keisan::ekf_ball::getcov);  
}