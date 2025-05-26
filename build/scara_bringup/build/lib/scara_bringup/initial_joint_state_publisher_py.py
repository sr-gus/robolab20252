#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, DurabilityPolicy
import time

def main(args=None):
    rclpy.init(args=args)

    # Creamos el nodo sin parámetros en el constructor
    node = Node('initial_joint_state_publisher')

    # Declaramos use_sim_time si no existe
    try:
        node.declare_parameter('use_sim_time', True)
    except ParameterAlreadyDeclaredException:
        pass

    if node.get_parameter('use_sim_time').value:
        node.get_logger().info('initial_joint_state usando /clock para sim time')

    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    pub = node.create_publisher(JointState, '/joint_states', qos)

    # Esperamos a que clock comience
    time.sleep(5)

    js = JointState()
    js.header.stamp = node.get_clock().now().to_msg()
    js.name = ['link_1_joint', 'link_2_joint', 'link_3_joint']
    js.position = [0.0, 0.0, 0.0]
    pub.publish(js)
    node.get_logger().info('Publicado estado inicial de SCARA')

    time.sleep(0.1)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
