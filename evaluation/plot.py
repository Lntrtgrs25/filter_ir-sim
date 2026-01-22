import matplotlib.pyplot as plt

def plot_trajectory(gt, kf, ekf, ekf2):
    plt.plot(gt[:,0], gt[:,1], 'k-', label='Ground Truth')
    plt.plot(kf[:,0], kf[:,1], 'r--', label='KF')
    plt.plot(ekf[:,0], ekf[:,1], 'b-.', label='EKF')
    plt.plot(ekf2[:,0], ekf2[:,1], 'g:', label='EKF2')
    plt.legend()
    plt.axis('equal')
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.grid()

def plot_error(err_kf, err_ekf):
    plt.plot(err_kf, label='KF error')
    plt.plot(err_ekf, label='EKF error')
    plt.xlabel('time step')
    plt.ylabel('error (m)')
    plt.legend()
    plt.grid()
