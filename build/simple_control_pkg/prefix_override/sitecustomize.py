import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/saun/ctrl_mr_dynamic/install/simple_control_pkg'