import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/robousr/Robotica2025-2/robolab20252/ExamenParcial2/ros2_ws/install/scara_control'
