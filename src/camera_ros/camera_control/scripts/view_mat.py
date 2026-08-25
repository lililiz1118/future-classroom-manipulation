import scipy.io as scio

meta_path = "/home/jt001/graspnet-baseline/doc/d405_data/water/meta.mat"
meta = scio.loadmat(meta_path)

print(meta)
# 查看所有 key
# print("文件内容 keys:", meta.keys())

# # 查看相机内参
# print("Intrinsic matrix:")
# print(meta['intrinsic_matrix'])

# # 查看深度比例因子
# print("Depth factor:")
# print(meta['factor_depth'])
