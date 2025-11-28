import numpy as np
from scipy.spatial.transform import Rotation as R

# 假设这是你已有的初始旋转矩阵 (继续使用之前的例子)
R_initial_matrix = np.array([
    [ 0.14391817, -0.97053623,  0.19325347],
    [-0.2771057,  -0.226998,   -0.93364036],
    [ 0.95,        0.08081617, -0.30161026]
])

# 1. 创建一个代表“绕基坐标系Y轴旋转180度”的旋转对象
# 注意：这里我们将轴从 'z' 改为了 'y'
rotation_y_180 = R.from_euler('y', 180, degrees=True)

# 2. 将你已有的矩阵也转换为一个旋转对象
initial_rotation = R.from_matrix(R_initial_matrix)

# 3. 进行旋转组合 (左乘操作)
final_rotation = rotation_y_180 * initial_rotation

# 4. 从旋转对象中获取最终的旋转矩阵
R_final_matrix = final_rotation.as_matrix()


print("--- 初始旋转矩阵 (R_initial) ---")
print(np.round(R_initial_matrix, 4))
print("\n--- 绕基坐标系Y轴旋转180度后的矩阵 (R_final) ---")
print(np.round(R_final_matrix, 4))

# 验证一下：绕Y轴旋转180度，会使X和Z坐标变号，Y坐标不变。
# 我们可以看到，新矩阵的第一列和第三列，分别是原矩阵第一列和第三列的相反数。