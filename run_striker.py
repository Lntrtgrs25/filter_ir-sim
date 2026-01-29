# import irsim
# env = irsim.make('scenarios_player/s0_check.yaml')

# for i in range(5000):

#     env.step()
#     env.render(0.05)

#     if env.done():
#         break

# env.end()

import csv
import irsim
import math
import numpy as np
import ekf_module  

csv_file = open('data_simulasi.csv', mode='w', newline='')
csv_writer = csv.writer(csv_file)

csv_writer.writerow(['true_x', 'true_y', 'noisy_x', 'noisy_y', 'ekf_x', 'ekf_y'])

env = irsim.make('scenarios_striker/s4_rdiagonal.yaml')

ekf = ekf_module.EKFBall() 

bola_initial_x = 5.0
bola_initial_y = 3.0
bola_initial_v = 0.3  
bola_initial_theta = -1.57  
ekf.init(bola_initial_x, bola_initial_y, bola_initial_v, bola_initial_theta)

initial_rtheta = 0.0

while True:
    env.step()

    robot = env.robot_list[0]
    bola  = env.obstacle_list[0]

    # rx, ry, rtheta = robot.state
    # bx, by, _      = bola.state

    rx = robot.state[0].item()
    ry = robot.state[1].item()
    rtheta = robot.state[2].item()

    bx = bola.state[0].item()
    by = bola.state[1].item()

    view_rtheta = math.atan2(by - ry, bx - rx)

    angle_error = view_rtheta - rtheta
    angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

    bx_relative = bx - rx
    by_relative = by - ry

    pos_noise_std = 0.04                                            #tambahan buat atur noise 
    # angle_noise_std = 0.05

    # fake measurement
    z_rel = np.array([
        bx_relative + np.random.normal(0, pos_noise_std),
        by_relative + np.random.normal(0, pos_noise_std)
    ])

    # noisy_rtheta = rtheta + np.random.normal(0, angle_noise_std) #tambahan noise 
    z = [float(rx + z_rel[0]), float(ry + z_rel[1])]
    
    print(f"non-EKF state= {z}")
    print(f"non-EKF rel state = {z_rel}")

#EKF calling
    dt = 0.1
    ekf.predict(dt)
    ekf.update(z) 

    estimated_pos = ekf.getPosition()  
    estimated_vel = ekf.getVelocity()
    estimated_state = ekf.getstate()
    print(f"EKF state: x={estimated_pos[0]:.3f}, y={estimated_pos[1]:.3f}")
    # print(f"EKF estimate vel: vx={estimated_vel[0]:.3f}, vy={estimated_vel[1]:.3f}")

# Simpan data ke CSV
    csv_writer.writerow([
        bx, by,         
        z[0], z[1],                
        estimated_pos[0], estimated_pos[1]  
    ])

    angular_gain = 1.0
    linear_vel = 0.0
    angular_vel = angular_gain * angle_error
    robot.set_velocity([linear_vel, angular_vel])
    
    env.render(0.05)
    if env.done(): break

csv_file.close()
env.end()
print("Data simulasi berhasil disimpan ke data_simulasi.csv")