import cv2
import numpy as np
from robot import Robot
from scipy.spatial.transform import Rotation
import rotations


tcp_host_ip = '192.168.1.19'  # IP and port to robot arm as TCP client (UR5)
tcp_port = 30002
workspace_limits = np.asarray([[-0.653, -0.2], [-0.224, 0.5], [-0.35, 0.326]])  # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)

# rpy转旋转矢量
def rpy2vec(rolldeg, pitchdeg, yawdeg):
    roll = np.deg2rad(rolldeg)
    pitch = np.deg2rad(pitchdeg)
    yaw = np.deg2rad(yawdeg)

    rotation_matrix = Rotation.from_euler('xyz', [yaw, pitch, roll]).as_matrix()
    rotation_vector = Rotation.from_matrix(rotation_matrix).as_rotvec()
    return rotation_vector


# 旋转矢量转rpy
def vec2rpy(rx, ry, rz):
    rotation_vector = np.array([rx, ry, rz])
    rotation_matrix = Rotation.from_rotvec(rotation_vector).as_matrix()
    rpy_angles = Rotation.from_matrix(rotation_matrix).as_euler('xyz')
    roll, pitch, yaw = np.rad2deg(rpy_angles)
    # print('rpy=',roll,pitch,yaw)
    return [roll, pitch, yaw]


if __name__ == "__main__":
    '''
    # Initialize robot and move to home pose
    robot = Robot(False, None, None, workspace_limits,
                tcp_host_ip, tcp_port, None, None,
                False, None, None)
    # robot.go_home()
    robot.set_ur_tcp()
    current_pos = robot.tcp_urx.getl()[:3]
    current_rotv = np.array(robot.tcp_urx.getl()[3:6])
    current_mat = cv2.Rodrigues(current_rotv)[0]
    current_quat = rotations.mat2quat(current_mat)
    current_euler = rotations.quat2euler(current_quat)
    print(current_pos,current_quat,current_euler)
    position=[-0.467, 0.358, 0.272]
    quat=[ 0.5 ,-0.5 , 0.5 , 0.5]
    new_mat = rotations.quat2mat(quat)
    orientation = np.transpose(cv2.Rodrigues(new_mat)[0])[0]
    robot.move_to(position, orientation,tool_acc=0.1, tool_vel=0.2)
    robot.set_gripper(84)
    '''

    # 测试一下精度
    robot = Robot(False, None, None, workspace_limits,
                  tcp_host_ip, tcp_port, None, None,
                  False, None, None)
    # robot.set_ur_tcp()
    current_pos = robot.tcp_urx.getl()[:3]
    current_rotv = np.array(robot.tcp_urx.getl()[3:6])
    current_mat = cv2.Rodrigues(current_rotv)[0]
    current_quat = rotations.mat2quat(current_mat)
    current_euler = rotations.quat2euler(current_quat)
    print(current_pos, '\n', current_rotv, '\n', current_quat)

    # pos [-0.3713749756942525, 0.031154233492579717, 0.24795006567528638]
    # quat [ 0.00833616  0.68928081 -0.00361613 -0.7244373 ]
    position = [-0.3713749756942525, 0.031154233492579717, 0.24795006567528638]
    quat = [0.00833616, 0.68928081, -0.00361613, -0.7244373 ]
    new_mat = rotations.quat2mat(quat)
    orientation = np.transpose(cv2.Rodrigues(new_mat)[0])[0]
    robot.move_to(position, orientation, tool_acc=0.1, tool_vel=0.2)
    robot.set_gripper(0)

    # position = [-0.285, -0.094, 0.278]
    # quat = [0.51020055, 0.48953438, -0.48774698, -0.51201013]
    # new_mat = rotations.quat2mat(quat)
    # orientation = np.transpose(cv2.Rodrigues(new_mat)[0])[0]
    # robot.move_to(position, orientation,tool_acc=0.1, tool_vel=0.2)
    # robot.set_gripper(10)

    # pipe_position = [0,0.7,1.0-0.635]
    # quat = [0,0,1,0]
    # new_mat = rotations.quat2mat(quat)
    # orientation = np.transpose(cv2.Rodrigues(new_mat)[0])[0]
    # # # # orientation = rotations.quat_rot_vec(quat)
    # # # # orientation = [0.0,-1.57,0.0]#[float(grasp_pose[3]), float(grasp_pose[4]), float(grasp_pose[5])]
    # joint_config = [0.0, -(95.0/360.0)*2*np.pi, (128.0/360.0)*2*np.pi, -(218.0/360.0)*2*np.pi, -(90.0/360.0)*2*np.pi, -(90.0/360.0)*2*np.pi]

    # robot.move_to(pipe_position, orientation,tool_acc=0.01, tool_vel=0.1)

    # # current_rotv = robot.get_pose()[3:]
    # # current_rotv = np.array(current_rotv)
    # # current_mat = cv2.Rodrigues(current_rotv)[0]
    # # current_euler = rotations.mat2euler(current_mat) 
    # # print("current_euler:", current_euler)
    # # print("current_euler1:", vec2rpy(current_rotv[0], current_rotv[1], current_rotv[2]))

    # # euler = np.array([20,-90,0])
    # # rotmat = rotations.euler2mat(euler)
    # # print("rotmat:",rotmat)
    # # rotvec = cv2.Rodrigues(rotmat)[0]
    # # orientation = [rotvec[0][0],rotvec[1][0],rotvec[2][0]]
    # # print("rotvec:",orientation)
    # # print("rpy2vec:",rpy2vec(20,-90,0))
    # rolldeg = random.randint(-30,30)
    orientation = rpy2vec(90, 0, 90)
    # print(orientation)
    # # current_rotv = robot.get_pose()[3:]
    # # current_euler  = vec2rpy(current_rotv[0], current_rotv[1], current_rotv[2])
    # # new_euler = current_euler + np.array([20,0,0])
    # # orientation = rpy2vec(new_euler[0],new_euler[1],new_euler[2])

    # # orientation = rpy2vec(0,-100,0)
    # print("orientation=",orientation)
    # # # new_deg = vec2rpy(orientation[0], orientation[1], orientation[2])
    # # # print("test: ", new_deg)
    # position = [-0.795,-0.068,0.320]

    # robot.move_to(position, orientation,tool_acc=0.1, tool_vel=0.1)

    # current_rotv = robot.get_pose()[3:]
    # current_euler  = vec2rpy(current_rotv[0], current_rotv[1], current_rotv[2])
    # print("current_euler:", current_euler)
    # new_euler = current_euler + np.array([-rolldeg,0,0])
    # orientation = rpy2vec(new_euler[0],new_euler[1],new_euler[2])
    # robot.move_to(position, orientation,tool_acc=0.1, tool_vel=0.1)

    # current_rotv = robot.get_pose()[3:]
    # current_euler  = vec2rpy(current_rotv[0], current_rotv[1], current_rotv[2])
    # print("current_euler:", current_euler)

    # # current_rotv = robot.get_pose()[3:]
    # # current_rotv = np.array(current_rotv)
    # # print("current_euler:", vec2rpy(current_rotv[0], current_rotv[1], current_rotv[2]))
    # # pose = robot.get_pose()
    # # rotation_vector = pose[3:]
    # # print('vector=',rotation_vector)
    # # rpy_angles = vec2rpy(rotation_vector[0],rotation_vector[1],rotation_vector[2])

    # # robot.set_gripper(55)

    # # position = [-0.600,-0.068,0.220]
    # # robot.move_to(position, orientation,tool_acc=0.01, tool_vel=0.01)

    # print("roll=",rolldeg)
    # # robot.restart_real()
