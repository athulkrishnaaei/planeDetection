import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/athul/Hiwi/submodules/planeDetection/install/pointcloud_plane_detection'
