# EKF filtering simulation with IR-SIM
This project implements an **Extended Kalman Filter (EKF)** to track a noisy ball's trajectory in the **ir-sim** robotic simulator. The system features a high-performance C++ backend for state estimation, interfaced with Python via **pybind11** for robot control and simulation.

## Technical Architecture
   - Sensor Layer: Captures (x,y) ball coordinates from ir-sim with added Gaussian noise to simulate real-world sensor limitations.
   - Estimation Layer (C++):
        - Model: Constant Velocity (CV) kinematic model.
        - State Vector: X=[x,y,v,θ]T.
        - Tuning: Utilizes Covariance Matrices Q (Process Noise) and R (Measurement Noise) to balance model trust vs. sensor trust.
