import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

import irsim
import numpy as np
import math


class SimRosBridge(Node):

    def __init__(self):
        super().__init__('ros_ir_sim')

        self.env = irsim.make('scenarios_player/s1_basic.yaml')

        self.pub = self.create_publisher(Point, '/ball/observation', 10)

        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.step)

        # camera parameters
        self.fov = math.radians(120.0)   # 120 deg
        self.max_range = 5.0             # meter
        self.noise_std = 0.05             # meter


    def step(self):
        self.env.step()
        self.env.render(0.05)

        robot = self.env.robot_list[0]
        ball  = self.env.obstacle_list[0]

        # rx, ry, rtheta = robot.state
        # bx, by, _      = ball.state
        rx = float(robot.state[0])
        ry = float(robot.state[1])
        rtheta = float(robot.state[2])

        bx = float(ball.state[0])
        by = float(ball.state[1])

        # transform to robot frame
        dx = bx - rx
        dy = by - ry

        # rotate ke frame robot
        x_rel =  math.cos(-rtheta)*dx - math.sin(-rtheta)*dy
        y_rel =  math.sin(-rtheta)*dx + math.cos(-rtheta)*dy

        distance = math.hypot(x_rel, y_rel)
        angle = math.atan2(y_rel, x_rel)

        # cek fov + range
        if distance > self.max_range:
            return

        if abs(angle) > self.fov / 2.0:
            return

        x_meas = x_rel + np.random.normal(0, self.noise_std)
        y_meas = y_rel + np.random.normal(0, self.noise_std)

        #publish measurement
        msg = Point()
        msg.x = x_meas
        msg.y = y_meas
        msg.z = 0.0

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SimRosBridge()
    rclpy.spin(node)
    node.env.end()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
