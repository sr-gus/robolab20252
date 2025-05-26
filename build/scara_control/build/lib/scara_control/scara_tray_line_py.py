#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from math import cos, sin, acos, atan2, sqrt

class ScaraTrayLineNode(Node):
    def __init__(self):
        super().__init__('scara_tray_line_node')

        # Declaramos use_sim_time si no lo estaba
        try:
            self.declare_parameter('use_sim_time', True)
        except ParameterAlreadyDeclaredException:
            pass

        if self.get_parameter('use_sim_time').value:
            self.get_logger().info('Usando /clock para sim time')

        self.joints_ = ['link_1_joint', 'link_2_joint', 'link_3_joint']
        self.total_time = 10.0

        # start_time queda vacío hasta que clock > 0
        self.start_time = None
        self.last_publish_time = None

        # Publishers
        self.state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.traj_pub  = self.create_publisher(
            JointTrajectory,
            '/scara_trajectory_controller/joint_trajectory',
            10
        )

        # Timer a 50 Hz
        self.create_timer(1.0/50.0, self.timer_callback)
        self.get_logger().info('Nodo SCARA activo: tray. línea recta, 50 Hz')

    def timer_callback(self):
        now = self.get_clock().now()

        # Primera llamada: inicializamos start_time cuando clock > 0
        if self.start_time is None:
            if now.nanoseconds == 0:
                return
            self.start_time = now
            self.last_publish_time = now
            return

        # Verificar si ha pasado suficiente tiempo desde la última publicación
        if self.last_publish_time is not None:
            time_since_last = (now - self.last_publish_time).nanoseconds * 1e-9
            if time_since_last < 0.02:  # 50 Hz = 0.02 segundos
                return

        elapsed = (now - self.start_time).nanoseconds * 1e-9  # segundos
        t = min(elapsed / self.total_time, 1.0)

        # Cinemática inversa
        sol = invk_sol(
            t,
            x_in=0.6, y_in=0.1, theta_in=0.0,
            x_fin=0.4, y_fin=-0.4, theta_fin=-0.75
        )

        # 1) JointState (RViz & broadcaster)
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = self.joints_
        js.position = sol
        self.state_pub.publish(js)

        # 2) JointTrajectory (controller)
        traj = JointTrajectory()
        traj.joint_names = self.joints_
        pt = JointTrajectoryPoint()
        pt.positions = sol
        pt.velocities = [0.0] * len(self.joints_)  # Añadimos velocidades
        pt.accelerations = [0.0] * len(self.joints_)  # Añadimos aceleraciones
        pt.time_from_start = Duration(
            sec=int(elapsed),
            nanosec=int((elapsed % 1) * 1e9)
        )
        traj.points = [pt]
        self.traj_pub.publish(traj)

        self.last_publish_time = now

        if t >= 1.0:
            self.get_logger().info('Trayectoria completada')
            # opcional: self.destroy_node()

def invk_sol(u, x_in, y_in, theta_in, x_fin, y_fin, theta_fin):
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
