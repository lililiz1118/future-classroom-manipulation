#include <iostream>
#include <unistd.h>
#include "dh_gripper_factory.h"

#include "ros/ros.h"
#include "dh_gripper_msgs/GripperCtrl.h"
#include "dh_gripper_msgs/GripperState.h"
#include "dh_gripper_msgs/GripperRotCtrl.h"
#include "dh_gripper_msgs/GripperRotState.h"

#include "sensor_msgs/JointState.h"

std::string _gripper_ID;
std::string _gripper_model;
std::string _gripper_connect_port;
std::string _gripper_Baudrate;
DH_Gripper *_gripper;

void update_gripper_control(const dh_gripper_msgs::GripperCtrl::ConstPtr& msg)
{
    // 保持不变...
     if(msg->initialize)
    {
      _gripper->Initialization();  
    }
    else
    {
        ROS_INFO("speed:[%f],force: [%f], position: [%f]", msg->speed,msg->force, msg->position);
        _gripper->SetTargetSpeed((int)msg->speed);
        _gripper->SetTargetForce((int)msg->force);
        _gripper->SetTargetPosition((int)msg->position);
    }
}

void update_rotation_control(const dh_gripper_msgs::GripperRotCtrl::ConstPtr& msg)
{
    // 保持不变...
    ROS_INFO("r_speed:[%f],r_force: [%f], r_angle: [%f]", msg->speed,msg->force, msg->angle);
    if(_gripper_model.find("RGI")!= _gripper_model.npos)
    {
        dynamic_cast<DH_RGI *>(_gripper)->SetTargetRotationTorque((int)msg->force); 
        dynamic_cast<DH_RGI *>(_gripper)->SetTargetRotationSpeed((int)msg->speed); 
        dynamic_cast<DH_RGI *>(_gripper)->SetTargetRotation((int)msg->angle); 

    }
    else if(_gripper_model.find("DH3_CAN")!= _gripper_model.npos)
    {
        dynamic_cast<DH_DH3_CAN *>(_gripper)->SetTargetRotation((int)msg->angle); 
    }
}

void update_gripper_state(dh_gripper_msgs::GripperState& msg)
{
    // 保持不变...
    static long seq = 0;
    msg.header.stamp = ros::Time::now();
    msg.header.seq =seq; 
    int tmp_state[5] = {0};
    _gripper->GetRunStates(tmp_state);
    if(tmp_state[0] == 1)
        msg.is_initialized = true;
    else
        msg.is_initialized = false;
        
    msg.grip_state      = tmp_state[1];
    msg.position        = tmp_state[2];
    msg.target_position = tmp_state[3];
    msg.target_force    = tmp_state[4];
    seq++;
}

void update_gripper_joint_state(sensor_msgs::JointState& msg)
{
    // 保持不变...
    static long seq = 0;
    msg.header.frame_id = "";
    msg.header.stamp = ros::Time::now();
    msg.header.seq = seq;
    
    msg.name.resize(1);
    msg.position.resize(1);


    int tmp_pos = 0;

    _gripper->GetCurrentPosition(tmp_pos);

    msg.position[0] = (1000-tmp_pos)/1000.0 * 0.637;
    msg.name[0] = "gripper_finger1_joint"; 

    seq++;
}

void update_rotation_state(dh_gripper_msgs::GripperRotState& msg)
{
    // 保持不变...
    static long seq = 0;
    msg.header.stamp = ros::Time::now();
    msg.header.seq =seq; 
    int tmp_state[9] = {0};
    _gripper->GetRunStates(tmp_state); 
    msg.rot_state       = tmp_state[5];
    msg.angle           = tmp_state[6];
    msg.target_angle    = tmp_state[7];
    msg.target_force    = tmp_state[8];
    seq++;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dh_gripper_driver");
    ros::NodeHandle nh;  // 全局节点句柄
    ros::NodeHandle pnh("~"); // 私有节点句柄用于参数

    // 获取参数 - 修正大小写
    pnh.param<std::string>("gripper_id", _gripper_ID, "1"); // 注意小写
    pnh.param<std::string>("gripper_model", _gripper_model, "AG95_MB");
    pnh.param<std::string>("connect_port", _gripper_connect_port, "/dev/ttyUSB0");
    pnh.param<std::string>("baudrate", _gripper_Baudrate, "115200"); // 注意小写
    
    ROS_INFO("Gripper_ID: %s", _gripper_ID.c_str());
    ROS_INFO("Gripper_model: %s", _gripper_model.c_str());
    ROS_INFO("Connect_port: %s", _gripper_connect_port.c_str());
    ROS_INFO("BaudRate: %s", _gripper_Baudrate.c_str());

    DH_Gripper_Factory* _gripper_Factory = new DH_Gripper_Factory();
    _gripper_Factory->Set_Parameter(atoi(_gripper_ID.c_str()), 
                                   _gripper_connect_port, 
                                   atoi(_gripper_Baudrate.c_str()));
    
    _gripper = _gripper_Factory->CreateGripper(_gripper_model);
    if(_gripper == NULL)
    {
        ROS_ERROR("Unsupported gripper model: %s", _gripper_model.c_str());
        delete _gripper_Factory;
        return -1;
    }   

    if(_gripper->open() < 0)
    {
        ROS_ERROR("Failed to open port: %s", _gripper_connect_port.c_str());
        delete _gripper_Factory;
        return -1;
    }

    // 初始化夹爪（带超时）
    int initstate = 0;
    _gripper->GetInitState(initstate);
    if(initstate != DH_Gripper::S_INIT_FINISHED)
    {
        ROS_INFO("Initializing gripper...");
        if(_gripper->Initialization())
            ROS_INFO("Initialization command acknowledged; waiting for hardware state");
        else
            ROS_WARN("Initialization command was not acknowledged; readiness gate will stop startup");
    }

    // 使用全局节点句柄创建发布者和订阅者
    ros::Publisher gripper_state_pub = nh.advertise<dh_gripper_msgs::GripperState>(
        "gripper/states", 10);
    
    ros::Publisher gripper_joint_state_pub = nh.advertise<sensor_msgs::JointState>(
        "gripper/joint_states", 10);
    
    ros::Subscriber grip_sub = nh.subscribe("gripper/ctrl", 10, 
                                          update_gripper_control);

    // 旋转控制（如果适用）
    ros::Publisher rot_state_pub;
    ros::Subscriber rot_sub;
    
    if(_gripper->GetGripperAxiNumber() == 2)
    {
        rot_state_pub = nh.advertise<dh_gripper_msgs::GripperRotState>(
            "gripper/rot_states", 10);
        
        rot_sub = nh.subscribe("gripper/rot_ctrl", 10, 
                             update_rotation_control);
    }

    ROS_INFO("Gripper driver ready. Publishing topics:");
    ROS_INFO("  - gripper/states");
    ROS_INFO("  - gripper/joint_states");
    if(_gripper->GetGripperAxiNumber() == 2) {
        ROS_INFO("  - gripper/rot_states");
    }

    ros::Rate loop_rate(50);  // 50Hz
    while(ros::ok())
    {
        // 发布夹爪状态
        dh_gripper_msgs::GripperState grip_state_msg;
        update_gripper_state(grip_state_msg);
        gripper_state_pub.publish(grip_state_msg);

        // 发布关节状态
        sensor_msgs::JointState joint_state_msg;
        update_gripper_joint_state(joint_state_msg);
        gripper_joint_state_pub.publish(joint_state_msg);

        // 发布旋转状态（如果适用）
        if(_gripper->GetGripperAxiNumber() == 2)
        {
            dh_gripper_msgs::GripperRotState rot_state_msg;
            update_rotation_state(rot_state_msg);
            rot_state_pub.publish(rot_state_msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    // 清理
    _gripper->close();
    delete _gripper_Factory;
    return 0;
}
