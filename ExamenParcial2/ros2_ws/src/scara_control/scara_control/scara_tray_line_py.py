#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from rclpy.duration import Duration as RosDuration
from math import cos, sin, acos, atan2, sqrt

class ScaraTrayLineNode(Node):
    def __init__(self):
        super().__init__('scara_tray_line_node')
        self.joints_ = ['link_1_joint', 'link_2_joint', 'link_3_joint']

        # Duración total de la trayectoria (segundos)
        self.total_time = 10.0

        # Marca de tiempo de inicio
        self.start_time = self.get_clock().now()

        # Publishers
        self.state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.traj_pub  = self.create_publisher(JointTrajectory, '/scara_trajectory_controller/joint_trajectory', 10)

        # Timer a 50 Hz
        self.timer = self.create_timer(1.0/50.0, self.timer_callback)
        self.get_logger().info('Nodo SCARA activo: tray. línea recta, 50 Hz')

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9  # en segundos
        t = min(elapsed / self.total_time, 1.0)

        # Cinemática inversa parametrizada por t∈[0,1]
        sol = invk_sol(t,
            x_in=0.6, y_in=0.1, theta_in=0,
            x_fin=0.4, y_fin=-0.4, theta_fin=-0.75
        )

        # Publica JointState para RViz
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = self.joints_
        js.position = sol
        self.state_pub.publish(js)

        # Publica JointTrajectory (opcional)
        traj = JointTrajectory()
        traj.joint_names = self.joints_
        pt = JointTrajectoryPoint()
        pt.positions = sol
        pt.time_from_start = Duration(sec=int(elapsed), nanosec=int((elapsed%1)*1e9))
        traj.points = [pt]
        self.traj_pub.publish(traj)

        # Cuando termine, detén el timer
        if t >= 1.0:
            self.get_logger().info('Trayectoria completada')
            self.timer.cancel()


def invk_sol(u, x_in, y_in, theta_in, x_fin, y_fin, theta_fin):
    # u ∈ [0,1]
    L1, L2, L3 = 0.5, 0.5, 0.3
    xP = x_in + u*(x_fin - x_in)
    yP = y_in + u*(y_fin - y_in)
    thetaP = theta_in + u*(theta_fin - theta_in)
    x3 = xP - L3*cos(thetaP)
    y3 = yP - L3*sin(thetaP)
    theta2 = acos((x3**2 + y3**2 - L1**2 - L2**2)/(2*L1*L2))
    beta = atan2(y3, x3)
    psi = acos((x3**2 + y3**2 + L1**2 - L2**2)/(2*L1*sqrt(x3**2 + y3**2)))
    theta1 = beta - psi
    theta3 = thetaP - theta1 - theta2
    return [theta1, theta2, theta3]


def main(args=None):
    rclpy.init(args=args)
    node = ScaraTrayLineNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
