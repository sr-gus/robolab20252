#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from math import cos, sin, acos, atan2, sqrt

def invk_sol(u, x_in, y_in, theta_in, x_fin, y_fin, theta_fin):
    L1, L2, L3 = 0.5, 0.5, 0.3
    # Interpolación lineal en el espacio cartesiano + orientación
    xP = x_in + u*(x_fin - x_in)
    yP = y_in + u*(y_fin - y_in)
    thetaP = theta_in + u*(theta_fin - theta_in)
    # Cálculo de posición del efector final (considerando L3)
    x3 = xP - L3*cos(thetaP)
    y3 = yP - L3*sin(thetaP)
    # Cinemática inversa clásica 3-R
    theta2 = acos((x3**2 + y3**2 - L1**2 - L2**2)/(2*L1*L2))
    beta   = atan2(y3, x3)
    psi    = acos((x3**2 + y3**2 + L1**2 - L2**2)/(2*L1*sqrt(x3**2 + y3**2)))
    theta1 = beta - psi
    theta3 = thetaP - theta1 - theta2
    return [theta1, theta2, theta3]

class ScaraTrayLineNode(Node):
    def __init__(self):
        super().__init__('scara_tray_line_node')

        # Declarar y usar sim time si corresponde
        try:
            self.declare_parameter('use_sim_time', True)
        except ParameterAlreadyDeclaredException:
            pass
        if self.get_parameter('use_sim_time').value:
            self.get_logger().info('Usando /clock para sim time')

        # Configuración de la trayectoria
        self.joints_ = ['link_1_joint', 'link_2_joint', 'link_3_joint']
        self.total_time = 10.0      # duración total de la trayectoria (s)
        self.start_time = None      # se inicializará al primer callback
        self.trajectory_completed = False  # para no repetir el mensaje

        # Publishers
        self.state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.traj_pub  = self.create_publisher(
            JointTrajectory,
            '/scara_trajectory_controller/joint_trajectory',
            10
        )

        # Timer a 50 Hz
        self.create_timer(1.0/50.0, self.timer_callback)
        self.get_logger().info('Nodo SCARA activo: trayectoria línea recta, 50 Hz')

    def timer_callback(self):
        now = self.get_clock().now()

        # Primer callback: esperar a que /clock avance
        if self.start_time is None:
            if now.nanoseconds == 0:
                return
            self.start_time = now
            return

        # Tiempo transcurrido y factor normalizado [0,1]
        elapsed = (now - self.start_time).nanoseconds * 1e-9
        t = min(elapsed / self.total_time, 1.0)

        # Cálculo de la solución de la cinemática inversa
        sol = invk_sol(
            t,
            x_in=0.6, y_in=0.1, theta_in=0.0,
            x_fin=0.4, y_fin=-0.4, theta_fin=-0.75
        )

        # 1) Publicar JointState (para RViz y broadcaster)
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = self.joints_
        js.position = sol
        self.state_pub.publish(js)

        # 2) Publicar JointTrajectory (para el controlador en Gazebo)
        traj = JointTrajectory()
        traj.joint_names = self.joints_
        pt = JointTrajectoryPoint()
        pt.positions = sol
        pt.time_from_start = Duration(
            sec=int(elapsed),
            nanosec=int((elapsed % 1) * 1e9)
        )
        traj.points = [pt]
        self.traj_pub.publish(traj)

        # Una vez completada la trayectoria, un solo log y detener nodo
        if t >= 1.0 and not self.trajectory_completed:
            self.get_logger().info('Trayectoria completada')
            self.trajectory_completed = True
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ScaraTrayLineNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
