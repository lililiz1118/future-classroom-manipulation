import scipy.io as scio
import numpy as np
import os

def update_meta_mat(original_meta_path, output_meta_path, fx, fy, ppx, ppy, factor_depth):
    """
    更新 meta.mat 中的相机内参和深度比例因子，并保存为新的文件。

    Args:
        original_meta_path (str): 原始 meta.mat 文件路径
        output_meta_path (str): 保存新 meta.mat 文件路径
        fx (float): 相机焦距 x
        fy (float): 相机焦距 y
        ppx (float): 光心 x
        ppy (float): 光心 y
        factor_depth (float): 深度比例因子（depth = raw / factor_depth）
    """
    if not os.path.exists(original_meta_path):
        print(f"错误：文件 {original_meta_path} 不存在！")
        return

    # 读取原始 meta.mat
    meta = scio.loadmat(original_meta_path)
    
    print("原始内参：")
    print(meta['intrinsic_matrix'])
    print("原始深度比例因子：", meta['factor_depth'])

    # 更新内参
    intrinsic_new = np.array([[fx, 0, ppx],
                              [0, fy, ppy],
                              [0,  0,  1]], dtype=np.float32)
    meta['intrinsic_matrix'] = intrinsic_new
    meta['factor_depth'] = np.array([factor_depth], dtype=np.float32)

    # 保存新文件
    scio.savemat(output_meta_path, meta)
    print(f"已保存修改后的 meta.mat 到 {output_meta_path}")
    print("新内参：")
    print(meta['intrinsic_matrix'])
    print("新深度比例因子：", meta['factor_depth'])


if __name__ == "__main__":
    # 示例参数，根据你的相机修改
    original_meta_path = "/home/jt001/graspnet-baseline/doc/d405_data/meta.mat"          # 原始 meta.mat 文件
    output_meta_path = "/home/jt001/graspnet-baseline/doc/d405_data/meta_new.mat"        # 保存新文件路径
    fx = 651.9721069335938
    fy = 650.947998046875
    ppx = 638.4763793945312
    ppy = 358.3318786621094
    factor_depth = 1000.000252621248                    # RealSense D400 系列常用 1000

    update_meta_mat(original_meta_path, output_meta_path, fx, fy, ppx, ppy, factor_depth)
