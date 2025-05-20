#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import time
from math import cos, sin, acos, atan2, sqrt

class ScaraTrayLineNode(Node):
    def __init__(self):
        super().__init__('scara_tray_line_node')
        self.lamda_ = 0
        self.Tiempo_ejec_ = 10
        self.joints_ = ['link_1_joint', 'link_2_joint', 'link_3_joint']

        # Publisher de trayectoria (puedes seguir usándolo)
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/scara_trajectory_controller/joint_trajectory', 10)

        # NUEVO: Publisher de estados de articulación
        self.state_pub = self.create_publisher(
            JointState, '/joint_states', 10)

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Scara activo, trayectoria linea recta')

    def timer_callback(self):
        # Calcula posiciones con cinemática inversa
        solucion = invk_sol(
            self.lamda_,
            x_in=0.1, y_in=0.6, theta_in=0,
            x_fin=0.3, y_fin=-0.6, theta_fin=1.57
        )

        # Publica JointTrajectory (opcional)
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joints_
        point = JointTrajectoryPoint()
        point.positions = solucion
        point.time_from_start = Duration(sec=1)
        traj_msg.points.append(point)
        self.traj_pub.publish(traj_msg)

        # --- NUEVO: Publica JointState para mover el modelo en RViz ---
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joints_
        js.position = solucion
        self.state_pub.publish(js)

        self.get_logger().info(f'Postura actual {solucion}')
        time.sleep(2)  # pausa para ver cada paso
        self.lamda_ = (self.lamda_ + 1) % (self.Tiempo_ejec_ + 1)

def invk_sol(param, x_in, y_in, theta_in, x_fin, y_fin, theta_fin):
    # ... tu función de cinemática inversa sin cambios ...
    # devuelve [theta_1, theta_2, theta_3]
    Tiempo_ejec_ = 10
    L1, L2, L3 = 0.5, 0.5, 0.3
    t = param / Tiempo_ejec_
    xP = x_in + t * (x_fin - x_in)
    yP = y_in + t * (y_fin - y_in)
    thetaP = theta_in + t * (theta_fin - theta_in)
    x3 = xP - L3 * cos(thetaP)
    y3 = yP - L3 * sin(thetaP)
    theta2 = acos((x3**2 + y3**2 - L1**2 - L2**2) / (2 * L1 * L2))
    beta = atan2(y3, x3)
    psi = acos((x3**2 + y3**2 + L1**2 - L2**2) / (2 * L1 * sqrt(x3**2 + y3**2)))
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
