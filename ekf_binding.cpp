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
    .def("getPosition", [](keisan::ekf_ball &self) {
        auto pos = self.getPosition();
        return std::vector<double>{pos[0][0], pos[1][0]};
    })
    .def("getVelocity", [](keisan::ekf_ball &self) {
        auto vel = self.getVelocity();
        return std::vector<double>{vel[0][0], vel[1][0]};
    })
    .def("getstate", [](keisan::ekf_ball &self) {
        auto s = self.getstate();
        return std::vector<double>{s[0][0], s[1][0], s[2][0], s[3][0]};
    })
    .def("getcov", [](keisan::ekf_ball &self) {
        auto P = self.getcov();
        std::vector<std::vector<double>> cov(4, std::vector<double>(4));
        for(int i=0; i<4; ++i)
            for(int j=0; j<4; ++j)
                cov[i][j] = P[i][j];
        return cov;
    });
}