#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from math import cos, sin, acos, atan2, sqrt

def poly35(s: float) -> float:
    """Perfil 3-4-5: suaviza velocidad y aceleración al inicio y fin."""
    return 10*s**3 - 15*s**4 + 6*s**5

def invk_solutions(u, x_in, y_in, theta_in, x_fin, y_fin, theta_fin):
    """
    Devuelve las dos soluciones (horario/antihorario) de la cinemática inversa
    para un SCARA 3R, interpolando entre las posturas inicial y final.
    """
    L1, L2, L3 = 0.5, 0.5, 0.3
    xP = x_in + u*(x_fin - x_in)
    yP = y_in + u*(y_fin - y_in)
    thetaP = theta_in + u*(theta_fin - theta_in)

    x3 = xP - L3*cos(thetaP)
    y3 = yP - L3*sin(thetaP)

    D = (x3**2 + y3**2 - L1**2 - L2**2) / (2*L1*L2)
    D = max(min(D, 1.0), -1.0)
    th2_pos = acos(D)
    th2_neg = -th2_pos
    beta = atan2(y3, x3)
    psi = acos((x3**2 + y3**2 + L1**2 - L2**2) / (2*L1*sqrt(x3**2 + y3**2)))

    th1_down = beta - psi
    th2_down = th2_pos
    th3_down = thetaP - th1_down - th2_down
    sol_down = [th1_down, th2_down, th3_down]

    th1_up = beta + psi
    th2_up = th2_neg
    th3_up = thetaP - th1_up - th2_up
    sol_up = [th1_up, th2_up, th3_up]

    return sol_down, sol_up

class ScaraTrayLineNode(Node):
    def __init__(self):
        super().__init__('scara_tray_line_node')

        params = [
            ('use_sim_time', True),
            ('x_in',       0.6),  ('y_in',       0.1),  ('theta_in',  0.0),
            ('x_fin',      0.4),  ('y_fin',     -0.4),  ('theta_fin', -0.75),
            ('total_time', 10.0)
        ]
        for name, default in params:
            try:
                self.declare_parameter(name, default)
            except ParameterAlreadyDeclaredException:
                pass

        p = self.get_parameters([n for n,_ in params])
        use_sim    = p[0].value
        self.x_in  = p[1].value
        self.y_in  = p[2].value
        self.theta_in = p[3].value
        self.x_fin = p[4].value
        self.y_fin = p[5].value
        self.theta_fin = p[6].value
        self.total_time = p[7].value

        if use_sim:
            self.get_logger().info('Usando /clock para sim time')

        self.joints_ = ['link_1_joint','link_2_joint','link_3_joint']
        self.start_time = None
        self.trajectory_completed = False
        self.last_sol = [0.0, 0.0, 0.0]

        self.state_pub = self.create_publisher(JointState,        '/joint_states', 10)
        self.traj_pub  = self.create_publisher(JointTrajectory, '/scara_trajectory_controller/joint_trajectory', 10)

        self.create_timer(1.0/50.0, self.timer_callback)
        self.get_logger().info(
            f'TrayLine: pose in ({self.x_in},{self.y_in},{self.theta_in}) → '
            f'pose fin ({self.x_fin},{self.y_fin},{self.theta_fin}) en {self.total_time}s'
        )

    def timer_callback(self):
        now = self.get_clock().now()

        if self.start_time is None:
            if now.nanoseconds == 0:
                return
            self.start_time = now
            return

        elapsed = (now - self.start_time).nanoseconds * 1e-9
        s = min(elapsed / self.total_time, 1.0)
        u = poly35(s)

        sol_down, sol_up = invk_solutions(
            u,
            self.x_in, self.y_in, self.theta_in,
            self.x_fin, self.y_fin, self.theta_fin
        )

        d_down = sum(abs(sol_down[i] - self.last_sol[i]) for i in range(3))
        d_up   = sum(abs(sol_up[i]   - self.last_sol[i]) for i in range(3))
        chosen = sol_down if d_down < d_up else sol_up
        self.last_sol = chosen

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = self.joints_
        js.position = chosen
        self.state_pub.publish(js)

        traj = JointTrajectory()
        traj.joint_names = self.joints_
        pt = JointTrajectoryPoint()
        pt.positions = chosen
        pt.time_from_start = Duration(sec=int(elapsed), nanosec=int((elapsed%1)*1e9))
        traj.points = [pt]
        self.traj_pub.publish(traj)

        if s >= 1.0 and not self.trajectory_completed:
            direction = 'down' if d_down < d_up else 'up'
            self.get_logger().info(f'Trayectoria completada '
                                   f'(perfil 3-4-5, ruta {direction})')
            self.trajectory_completed = True
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ScaraTrayLineNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
