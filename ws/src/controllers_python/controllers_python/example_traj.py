import rclpy
import numpy as np
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ExampleTraj(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self._HOME = [np.deg2rad(0), np.deg2rad(40),
                     np.deg2rad(30), np.deg2rad(-30)]
        self._beginning = self.get_clock().now()
        self._publisher = self.create_publisher(JointTrajectory, 'joint_cmds', 10)
        timer_period = 0.04  # seconds
        self._timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now()
        msg = JointTrajectory()
        msg.header.stamp = now.to_msg()

        dt = (now - self._beginning).nanoseconds * (1e-9)
        
        point = JointTrajectoryPoint()
        point.positions = [self._HOME[0] + 0.1 * np.pi * np.sin(2 * np.pi / 10.0 * dt),
                           self._HOME[1] + 0.1 * np.pi * np.sin(2 * np.pi / 10.0 * dt),
                           self._HOME[2] + 0.25 * np.pi * (np.sin(2 * np.pi / 10.0 * dt) - 1),
                           self._HOME[3] + 0.25 * np.pi * np.sin(2 * np.pi / 10.0 * dt),
                           0.5 * np.sin(2 * np.pi / 10.0 * dt) + 0.5
                           ]
        msg.points = [point]

        self._publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)

    example_traj = ExampleTraj()

    rclpy.spin(example_traj)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    example_traj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()