#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, DurabilityPolicy
import time

def main(args=None):
    rclpy.init(args=args)
    node = Node('initial_joint_state_publisher')

    # QoS latch: guarda el último mensaje para futuros subscribers
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    pub = node.create_publisher(JointState, '/joint_states', qos)

    # Armar y publicar el estado [0,0,0]
    js = JointState()
    js.header.stamp = node.get_clock().now().to_msg()
    js.name = ['link_1_joint', 'link_2_joint', 'link_3_joint']
    js.position = [0.0, 0.0, 0.0]
    pub.publish(js)
    node.get_logger().info('Publicado estado inicial de SCARA')

    # Dale un instante al middleware para asegurarse de la publicación
    time.sleep(0.1)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
