启动底盘，雷达，顶部相机节点
roslaunch tracer_bringup tracer_robot_base.launch
启动ur机械臂
roslaunch tracer_bringup tracer_ur_bringup.launch

机械臂二次上电
tra_ur3_power_on

释放刹车
tra_ur3_brake_release

加载程序
tra_ur3_load

执行程序
tra_ur3_play

抓取控制代码
手动将graspnet输出粘贴到输入变量中
~/tracer_ws/src/my_ur_control/scripts/ur3_grab.py（如果执行有问题，ctrl c重新执行）

拍照程序
~/tracer_ws/src/camera_ros/camera_control/scripts/get_photo.py
保存在
~/tracer_ws/src/camera_ros/images

graspnet
~/graspnet-baseline/doc/d405_data/cola 输入文件夹
输入四个文件，color，depth由相机拍，workspace_mask由sam给出，meta.mat固定

sam2
输入文件夹~/Grounded-SAM-2/notebooks/images/my_images
输出文件加~/Grounded-SAM-2/outputs/test_output


关电只关底盘和机械臂，其他不用关，机械臂关电中间的绿色按钮

机械臂开电先按左边的绿色按钮，过大概一分钟听到连续几声动静后再开机械臂节点

