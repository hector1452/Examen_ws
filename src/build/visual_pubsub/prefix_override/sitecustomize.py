import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hector/workspaces/clase_4_ws/src/install/visual_pubsub'
