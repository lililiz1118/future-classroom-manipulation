import urx
import time
import numpy as np

from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper


class Robot(object):
    def __init__(self, is_sim, obj_mesh_dir, num_obj, workspace_limits, tcp_host_ip, tcp_port,
                 rtc_host_ip, rtc_port, is_testing, test_preset_cases, test_preset_file):

        self.is_sim = is_sim
        self.workspace_limits = workspace_limits

       
        # Connect to robot client
        self.tcp_host_ip = tcp_host_ip
        self.tcp_port = tcp_port
        # self.tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Connect as real-time client to parse state data
        self.rtc_host_ip = rtc_host_ip
        self.rtc_port = rtc_port

        # Default home joint configuration
        # self.home_joint_config = [-np.pi, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0]
        # self.home_joint_config = [0.0, -(120.0/360.0)*2*np.pi, (120.0/360.0)*2*np.pi, -(90/360.0)*2*np.pi, -(90.0/360.0)*2*np.pi, 0.0]
        self.home_joint_config = [0.0, -(141.0/360.0)*2*np.pi, (120.0/360.0)*2*np.pi, -(70/360.0)*2*np.pi, -(93.0/360.0)*2*np.pi, 0.0]
        # Default joint speed configuration
        self.joint_acc = 2.1 # Safe: 1.4
        self.joint_vel = 1.1 # Safe: 1.05

        # Joint tolerance for blocking calls
        self.joint_tolerance = 0.005

        # Default tool speed configuration
        self.tool_acc = 1.2 # Safe: 0.5
        self.tool_vel = 0.25 # Safe: 0.2

        # Tool pose tolerance for blocking calls
        self.tool_pose_tolerance = 0.005

        # # Move robot to home pose
        self.open_gripper()
        # self.go_home()

        # Fetch RGB-D data from RealSense camera
        # from camera import Camera
        # self.camera = Camera()
        # self.cam_intrinsics = self.camera.intrinsics

        # Load camera pose (from running calibrate.py), intrinsics and depth scale
        # self.cam_pose = np.loadtxt('./real/camera_pose.txt', delimiter=' ')
        # self.cam_depth_scale = np.loadtxt('./real/camera_depth_scale.txt', delimiter=' ')

    def restart_real(self):
        self.go_home()
        #print("Please replace all object !")
        #input()
        print("Grasp goal object is OK!")


    def get_task_score(self):
        key_positions = np.asarray([[-0.625, 0.125, 0.0], # red
                                    [-0.625, -0.125, 0.0], # blue
                                    [-0.375, 0.125, 0.0], # green
                                    [-0.375, -0.125, 0.0]]) #yellow

        obj_positions = np.asarray(self.get_obj_positions())
        obj_positions.shape = (1, obj_positions.shape[0], obj_positions.shape[1])
        obj_positions = np.tile(obj_positions, (key_positions.shape[0], 1, 1))

        key_positions.shape = (key_positions.shape[0], 1, key_positions.shape[1])
        key_positions = np.tile(key_positions, (1 ,obj_positions.shape[1] ,1))

        key_dist = np.sqrt(np.sum(np.power(obj_positions - key_positions, 2), axis=2))
        key_nn_idx = np.argmin(key_dist, axis=0)

        return np.sum(key_nn_idx == np.asarray(range(self.num_obj)) % 4)


    def check_goal_reached(self):
        goal_reached = self.get_task_score() == self.num_obj
        return goal_reached

    def get_camera_data_init(self):
        # Get color and depth image from ROS service
        color_img, depth_img = self.camera.get_data()
        # color_img = self.camera.color_data.copy()
        # depth_img = self.camera.depth_data.copy()
        return color_img, depth_img

    def get_camera_data(self):
        color_img = []
        depth_img = []
        label_img = []
        visual_img = []
        # Get color and depth image from ROS service
        color_img, depth_img = self.camera.get_data()
        # color_img = self.camera.color_data.copy()
        # depth_img = self.camera.depth_data.copy()
        return color_img, depth_img, label_img, visual_img


    def close_gripper(self, asynch=False):
        self.tcp_urx = urx.Robot(self.tcp_host_ip)
        robotiq_gripper = Robotiq_Two_Finger_Gripper(self.tcp_urx)
        robotiq_gripper.close_gripper()
        self.tcp_urx.close()
        gripper_fully_closed = True
        if not asynch:
            time.sleep(1.5)

        return gripper_fully_closed


    def open_gripper(self, asynch=False):
        self.tcp_urx = urx.Robot(self.tcp_host_ip)
        robotiq_gripper = Robotiq_Two_Finger_Gripper(self.tcp_urx)
        robotiq_gripper.open_gripper()
        self.tcp_urx.close()
        if not asynch:
            time.sleep(1.5)

    # 设置手爪开合角度，percentage=[0,100]
    def set_gripper(self, percentage, asynch=False):
        self.tcp_urx = urx.Robot(self.tcp_host_ip)
        robotiq_gripper = Robotiq_Two_Finger_Gripper(self.tcp_urx)
        # percentage 0 for open, 100 for full close
        robotiq_gripper.gripper_action(int(percentage*255.0/100.0))
        self.tcp_urx.close()
        if not asynch:
            time.sleep(1.5)

    def move_to(self, tool_position, tool_orientation, tool_acc=0.01, tool_vel=0.01, tool_wait=True, tool_pose_tolerance=None):
        self.tcp_urx = urx.Robot(self.tcp_host_ip)
        self.tcp_urx.movel((tool_position[0],tool_position[1],tool_position[2],tool_orientation[0],tool_orientation[1],tool_orientation[2]), acc=tool_acc, vel=tool_vel, wait=tool_wait, threshold=tool_pose_tolerance)
        self.tcp_urx.close()

    def get_pose(self):
        self.tcp_urx = urx.Robot(self.tcp_host_ip)
        pose = self.tcp_urx.getl()
        self.tcp_urx.close()
        return pose
        
    def move_joints(self, joint_configuration, joint_acc=0.01, joint_vel=0.01, joint_wait=True, joint_tolerance=None):
        self.tcp_urx = urx.Robot(self.tcp_host_ip)
        self.tcp_urx.movej(joint_configuration, acc=joint_acc, vel=joint_vel, wait=joint_wait, threshold=joint_tolerance)
        self.tcp_urx.close()

    def go_home(self):
        self.move_joints(self.home_joint_config, self.joint_acc*0.5, self.joint_vel*0.5, True, self.joint_tolerance)


    # Primitives ----------------------------------------------------------

    def grasp(self, position, heightmap_rotation_angle, workspace_limits):
        print('Executing: grasp at (%f, %f, %f)' % (position[0], position[1], position[2]))
        # Compute tool orientation from heightmap rotation angle
        grasp_orientation = [1.0,0.0]
        if heightmap_rotation_angle > np.pi:
            heightmap_rotation_angle = heightmap_rotation_angle - 2*np.pi
        tool_rotation_angle = heightmap_rotation_angle/2
        tool_orientation = np.asarray([grasp_orientation[0]*np.cos(tool_rotation_angle) - grasp_orientation[1]*np.sin(tool_rotation_angle), grasp_orientation[0]*np.sin(tool_rotation_angle) + grasp_orientation[1]*np.cos(tool_rotation_angle), 0.0])*np.pi
        
        tool_orientation_angle = np.linalg.norm(tool_orientation)
        tool_orientation_axis = tool_orientation/tool_orientation_angle
        tool_orientation_rotm = utils.angle2rotm(tool_orientation_angle, tool_orientation_axis, point=None)[:3,:3]

        # Compute tilted tool orientation during dropping into bin
        tilt_rotm = utils.euler2rotm(np.asarray([-np.pi/4,0,0]))
        tilted_tool_orientation_rotm = np.dot(tilt_rotm, tool_orientation_rotm)
        tilted_tool_orientation_axis_angle = utils.rotm2angle(tilted_tool_orientation_rotm)
        tilted_tool_orientation = tilted_tool_orientation_axis_angle[0]*np.asarray(tilted_tool_orientation_axis_angle[1:4])

        # Attempt grasp
        position = np.asarray(position).copy()
        position[2] = max(position[2] + 0.132, workspace_limits[2][0] + 0.162)
        grasp_location_margin = 0.2

        self.open_gripper()
        self.move_to((position[0],position[1],position[2]+grasp_location_margin), tool_orientation, self.tool_acc*0.5, self.tool_vel*0.5, True, self.tool_pose_tolerance)
        self.move_to(position, tool_orientation, self.tool_acc*0.1, self.tool_vel*0.1, True, self.tool_pose_tolerance)
        self.close_gripper()

        # Check if flag is true (grasp might be successful)
        self.tcp_urx = urx.Robot(self.tcp_host_ip)
        robotiq_gripper = Robotiq_Two_Finger_Gripper(self.tcp_urx)
        #robotiq_gripper.set_detect_object_flag()
        gripper_open = int(self.tcp_urx.get_digital_out(1))
        self.tcp_urx.close()

        home_position = [-0.275, -0.109, 0.375]
        bin_position = [-0.489, -0.377, 0.375]

        # If gripper is open, drop object in bin and check if grasp is successful
        grasp_success = False
        if gripper_open:

            # Pre-compute blend radius
            blend_radius = min(abs(bin_position[1] - position[1])/2 - 0.01, 0.2)

            # Attempt placing
            self.move_to((position[0],position[1],bin_position[2]), tool_orientation, self.tool_acc, self.tool_vel, True, self.tool_pose_tolerance)
            self.move_to(bin_position, tilted_tool_orientation, self.tool_acc, self.tool_vel, True, self.tool_pose_tolerance)
            
            # If gripper width did not change before reaching bin location, then object is in grip and grasp is successful
            self.tcp_urx = urx.Robot(self.tcp_host_ip)
            robotiq_gripper = Robotiq_Two_Finger_Gripper(self.tcp_urx)
            robotiq_gripper.set_detect_object_flag()
            grasp_success = int(self.tcp_urx.get_digital_out(1))
            self.tcp_urx.close()

            self.open_gripper()
            self.move_to(home_position, tool_orientation, self.tool_acc*0.5, self.tool_vel*0.5, True, self.tool_pose_tolerance)

        else:
            self.move_to((position[0],position[1],position[2]+0.1), tool_orientation, self.tool_acc*0.5, self.tool_vel*0.5, True, self.tool_pose_tolerance)
            self.move_to(home_position, tool_orientation, self.tool_acc*0.5, self.tool_vel*0.5, True, self.tool_pose_tolerance)
            # self.open_gripper()
        return grasp_success

    def grasp_6d_pose(self, position, orientation, workspace_limits):
        print('Executing: grasp at (%f, %f, %f)' % (position[0], position[1], position[2]))
        # Compute tool orientation from heightmap rotation angle

        # Attempt grasp
        position = np.asarray(position).copy()
        orientation = np.asarray(orientation).copy()
        position[2] = max(position[2] + 0.132, workspace_limits[2][0] + 0.162)
        grasp_location_margin = 0.1

        self.open_gripper()
        position = [-0.34263,-0.03990,position[2]]
        self.move_to((position[0],position[1],position[2]+grasp_location_margin), orientation, self.tool_acc*0.5, self.tool_vel*0.5, True, self.tool_pose_tolerance)
        # self.move_to((-0.40099,0.03925,position[2]+grasp_location_margin), orientation, self.tool_acc*0.5, self.tool_vel*0.5, True, self.tool_pose_tolerance)
        input("enter to move to the object")
        self.move_to(position, orientation, self.tool_acc*0.5, self.tool_vel*0.5, True, self.tool_pose_tolerance)
        # self.move_to((-0.40099,0.03925,position[2]), orientation, self.tool_acc*0.1, self.tool_vel*0.1, True, self.tool_pose_tolerance)
        # input("enter to close gripper")
        self.close_gripper()
        # input("enter to moev to the height ")
        
        # Check if flag is true (grasp might be successful)
        self.tcp_urx = urx.Robot(self.tcp_host_ip)
        robotiq_gripper = Robotiq_Two_Finger_Gripper(self.tcp_urx)
        #robotiq_gripper.set_detect_object_flag()
        gripper_open = int(self.tcp_urx.get_digital_out(1))
        self.tcp_urx.close()

        home_position = [-0.2, -0.109, 0.375]
        bin_position = [-0.489, -0.377, 0.375]
        bin_orientation = [0.028675, 3.183690, -0.178802]
        # If gripper is open, drop object in bin and check if grasp is successful
        grasp_success = False
        # if gripper_open:
        #     # print("into gripper_open == true")
        #     # Pre-compute blend radius
        #     blend_radius = min(abs(bin_position[1] - position[1])/2 - 0.01, 0.2)

        #     # Attempt placing
        #     self.move_to((position[0],position[1],bin_position[2]), orientation, self.tool_acc, self.tool_vel, True, self.tool_pose_tolerance)
        #     self.move_to(bin_position, orientation, self.tool_acc, self.tool_vel, True, self.tool_pose_tolerance)
            
        #     # If gripper width did not change before reaching bin location, then object is in grip and grasp is successful
        #     self.tcp_urx = urx.Robot(self.tcp_host_ip)
        #     robotiq_gripper = Robotiq_Two_Finger_Gripper(self.tcp_urx)
        #     robotiq_gripper.set_detect_object_flag()
        #     grasp_success = int(self.tcp_urx.get_digital_out(1))
        #     self.tcp_urx.close()

        #     self.open_gripper()
        #     self.move_to(home_position, orientation, self.tool_acc*0.5, self.tool_vel*0.5, True, self.tool_pose_tolerance)

        # else:
            # print("into gripper_open == false")
        self.move_to((position[0],position[1],position[2]+0.1), orientation, self.tool_acc*0.5, self.tool_vel*0.5, True, self.tool_pose_tolerance)
        # input("enter to move to bin")
        banana_bin_position = [-0.34932,0.37031,-0.341]
        self.move_to((banana_bin_position[0],banana_bin_position[1],position[2]+0.1), orientation, self.tool_acc*0.5, self.tool_vel*0.5, True, self.tool_pose_tolerance)
        self.move_to((banana_bin_position[0],banana_bin_position[1],position[2]+0.05), orientation, self.tool_acc*0.5, self.tool_vel*0.5, True, self.tool_pose_tolerance)
        # self.move_to((banana_bin_position[0],banana_bin_position[1],position[2]+0.05), bin_orientation, self.tool_acc*0.5, self.tool_vel*0.5, True, self.tool_pose_tolerance)
        self.open_gripper()
        # input("enter to move to backon")
        self.move_to(home_position, orientation, self.tool_acc*0.5, self.tool_vel*0.5, True, self.tool_pose_tolerance)
        
        return grasp_success
   
    def robot_open_gripper(self):
        self.open_gripper()

    def robot_close_gripper(self):
        self.close_gripper()

if __name__ == '__main__':

    # --------------- Setup options ---------------
    tcp_host_ip = '192.168.1.19' # IP and port to robot arm as TCP client (UR5)
    tcp_port = 30002
    workspace_limits = np.asarray([[-0.653, -0.2], [-0.224, 0.5], [-0.35, 0.326]])  # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
# ---------------------------------------------


    robot = Robot(False, None, None, workspace_limits,
                tcp_host_ip, tcp_port, None, None,
                False, None, None)

    robot.set_gripper(0)
    

   
