import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.spatial.transform import Rotation as R

def visualize_transform(xyz, q, title="Coordinate Frame Visualization"):
    """
    可视化坐标变换
    输入:
        xyz: [x, y, z] 平移向量
        q:   [x, y, z, w] 四元数
        title: 图表标题
    """
    # 内部辅助函数：设置3D绘图的轴比例相等（避免图形畸变）
    def set_axes_equal(ax):
        x_limits = ax.get_xlim3d()
        y_limits = ax.get_ylim3d()
        z_limits = ax.get_zlim3d()

        x_range = abs(x_limits[1] - x_limits[0])
        x_middle = np.mean(x_limits)
        y_range = abs(y_limits[1] - y_limits[0])
        y_middle = np.mean(y_limits)
        z_range = abs(z_limits[1] - z_limits[0])
        z_middle = np.mean(z_limits)

        plot_radius = 0.5 * max([x_range, y_range, z_range])

        ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
        ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
        ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(title)

    # === 1. 绘制原点坐标系 (World Frame) ===
    # 原点位置
    o_origin = np.array([0, 0, 0])
    # 基向量 (单位矩阵)
    base_x = np.array([1, 0, 0])
    base_y = np.array([0, 1, 0])
    base_z = np.array([0, 0, 1])
    
    # 轴长度缩放
    scale = 0.1 

    # 绘制原点轴 (虚线表示原点)
    ax.quiver(o_origin[0], o_origin[1], o_origin[2], base_x[0], base_x[1], base_x[2], 
              length=scale, color='r', linestyle='--', label='Origin X')
    ax.quiver(o_origin[0], o_origin[1], o_origin[2], base_y[0], base_y[1], base_y[2], 
              length=scale, color='g', linestyle='--', label='Origin Y')
    ax.quiver(o_origin[0], o_origin[1], o_origin[2], base_z[0], base_z[1], base_z[2], 
              length=scale, color='b', linestyle='--', label='Origin Z')

    # === 2. 绘制变换后的子坐标系 (Target Frame) ===
    # 目标原点
    t_origin = np.array(xyz)
    # 计算旋转矩阵
    r = R.from_quat(q)
    rot_matrix = r.as_matrix()

    # 变换后的基向量
    t_x = rot_matrix @ np.array([1, 0, 0])
    t_y = rot_matrix @ np.array([0, 1, 0])
    t_z = rot_matrix @ np.array([0, 0, 1])

    # 绘制目标轴 (实线)
    # X轴 - 红色 (Red)
    ax.quiver(t_origin[0], t_origin[1], t_origin[2], t_x[0], t_x[1], t_x[2], 
              length=scale, color='r', linewidth=2, label='Target X')
    # Y轴 - 绿色 (Green)
    ax.quiver(t_origin[0], t_origin[1], t_origin[2], t_y[0], t_y[1], t_y[2], 
              length=scale, color='g', linewidth=2, label='Target Y')
    # Z轴 - 蓝色 (Blue)
    ax.quiver(t_origin[0], t_origin[1], t_origin[2], t_z[0], t_z[1], t_z[2], 
              length=scale, color='b', linewidth=2, label='Target Z')
    
    # 绘制连接线 (从原点到目标点)
    ax.plot([0, t_origin[0]], [0, t_origin[1]], [0, t_origin[2]], 'k:', alpha=0.5)

    # 设置标签
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    # 强制坐标轴比例一致
    set_axes_equal(ax)
    
    plt.show()