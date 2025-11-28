import urx
from scipy.spatial.transform import Rotation as R
import numpy as np
import collections.abc
collections.Iterable = collections.abc.Iterable

import urx
import numpy as np
from scipy.spatial.transform import Rotation as R

robot = urx.Robot("192.168.131.3")

# 获取 TCP 到 Base 的 Transform 对象
pose = robot.get_pose()  # Transform 对象

# 平移
tcp_pos = np.array(pose.pos)  # [x, y, z]

# 旋转（四元数）
tcp_quat = np.array(pose.orient)  # [x, y, z, w]



print("TCP position in base:", tcp_pos)
print("TCP quaternion in base:", tcp_quat)


