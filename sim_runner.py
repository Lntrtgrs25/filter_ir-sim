# import irsim
# env = irsim.make('scenarios_player/s0_check.yaml')

# for i in range(5000):

#     env.step()
#     env.render(0.05)

#     if env.done():
#         break

# env.end()

import irsim
import math
import numpy as np
import ekf_module  

env = irsim.make('scenarios_player/s0_check.yaml')

ekf = ekf_module.EKFBall() 

bola_initial_x = 5.3
bola_initial_y = 3.7
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

    # fake measurement
    z_rel = np.array([
        bx_relative + np.random.normal(0, 0.01),
        by_relative + np.random.normal(0, 0.01)
    ])
    z = [float(rx + z_rel[0]), float(ry + z_rel[1])]
    
    print(f"ball relative measurment = {z_rel}")
    print(f"absolute measurement = {z}")

#EKF calling
    dt = 0.1
    ekf.predict(dt)
    ekf.update(z) 

    estimated_pos = ekf.getPosition()  
    estimated_vel = ekf.getVelocity()
    estimated_state = ekf.getstate()
    print(f"EKF estimate pos: x={estimated_pos[0]:.3f}, y={estimated_pos[1]:.3f}")
    print(f"EKF estimate vel: vx={estimated_vel[0]:.3f}, vy={estimated_vel[1]:.3f}")

    angular_gain = 1.0
    linear_vel = 0.0
    angular_vel = angular_gain * angle_error
    robot.set_velocity([linear_vel, angular_vel])

    # rotation_threshold = 0.01  
    # if abs(angle_error) > rotation_threshold:
    #     fov_status = "moving"
    # else:
    #     fov_status = "static"

    # rotation_from_start = rtheta - initial_rtheta
    # rotation_from_start = math.atan2(math.sin(rotation_from_start), math.cos(rotation_from_start))  # Normalize to [-pi, pi]
    # print(f"FOV status: {fov_status}, Rotasi dari sudut awal: {math.degrees(rotation_from_start):.2f} derajat")

    env.render(0.05)

# nb : for debugging yaml

# import irsim
# import math
# import numpy as np

# env = irsim.make('scenarios_player/s0_check.yaml')

# # Initial robot angle
# initial_rtheta = 0.0

# while True:
#     env.step()

#     # ground truth
#     robot = env.robot_list[0]
#     bola  = env.obstacle_list[0]

#     rx = robot.state[0].item()
#     ry = robot.state[1].item()
#     rtheta = robot.state[2].item()

#     bx = bola.state[0].item()
#     by = bola.state[1].item()

#     view_rtheta = math.atan2(by - ry, bx - rx)

#     angle_error = view_rtheta - rtheta
#     angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

#     bx_relative = bx - rx
#     by_relative = by - ry

#     # fake measurement
#     z_rel = np.array([
#         bx_relative + np.random.normal(0, 0.01),
#         by_relative + np.random.normal(0, 0.01)
#     ])
#     z = np.array([rx + z_rel[0], ry + z_rel[1]])  # Convert ke absolut

#     print(f"ball relative measurment = {z_rel}")
#     print(f"absolute measurement = {z}")

#     env.render(0.05)
