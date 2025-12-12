import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ajr/ros2_ws/src/gyo_xj0_autonom_NB/install/gyo_xj0_autonom'
