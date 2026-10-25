import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/joel/HR/ros2_ws/src/Humanoid_Robot_training/install/tf_kinematics'
