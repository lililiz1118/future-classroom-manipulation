import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def visualize_pose(position, rotvec, axis_length=0.1):
    """
    可视化末端位姿，同时显示基坐标系
    :param position: 末端位置 [x, y, z]
    :param rotvec: 旋转矢量 [rx, ry, rz]
    :param axis_length: 坐标轴长度
    """
    # 旋转矩阵
    r = R.from_rotvec(rotvec)
    R_matrix = r.as_matrix()  # 3x3
    
    # 末端坐标系方向向量
    x_axis = R_matrix[:,0] * axis_length
    y_axis = R_matrix[:,1] * axis_length
    z_axis = R_matrix[:,2] * axis_length
    
    # 创建图形
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # 绘制基坐标系原点和坐标轴
    base_origin = [0, 0, 0]
    ax.scatter(*base_origin, color='k', s=50, label='Base Origin')
    ax.quiver(0,0,0, axis_length,0,0, color='r', normalize=True, label='Base X')
    ax.quiver(0,0,0, 0,axis_length,0, color='g', normalize=True, label='Base Y')
    ax.quiver(0,0,0, 0,0,axis_length, color='b', normalize=True, label='Base Z')
    
    # 绘制末端坐标系原点和坐标轴
    ox, oy, oz = position
    ax.scatter(ox, oy, oz, color='m', s=50, label='End Effector')
    ax.quiver(ox, oy, oz, x_axis[0], x_axis[1], x_axis[2], color='r', length=axis_length, normalize=True, label='End X')
    ax.quiver(ox, oy, oz, y_axis[0], y_axis[1], y_axis[2], color='g', length=axis_length, normalize=True, label='End Y')
    ax.quiver(ox, oy, oz, z_axis[0], z_axis[1], z_axis[2], color='b', length=axis_length, normalize=True, label='End Z')
    
    # 坐标轴范围
    all_points = np.array([base_origin, position])
    x_min, x_max = all_points[:,0].min()-0.2, all_points[:,0].max()+0.2
    y_min, y_max = all_points[:,1].min()-0.2, all_points[:,1].max()+0.2
    z_min, z_max = all_points[:,2].min()-0.2, all_points[:,2].max()+0.2
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_zlim(z_min, z_max)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('End Effector Pose Visualization')
    plt.legend()
    plt.show()



# 示例位置和旋转矢量
position = [0.11245955574373542, -0.14673268625373898, 0.3710247941971373]  # x, y, z (m)
rotvec = [0.017206413062495797, -2.5858785963688358, 1.7573729277409944]    # rx, ry, rz (rad)

visualize_pose(position, rotvec)
