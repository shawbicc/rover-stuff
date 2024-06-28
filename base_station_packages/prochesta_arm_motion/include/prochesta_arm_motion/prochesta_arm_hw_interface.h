#pragma once

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <angles/angles.h>
#include <prochesta_arm_motion/SixJoints.h>
#include <prochesta_arm_motion/JointSrv.h>

class ProchestaHWInterface : public hardware_interface::RobotHW {
    public:
        ProchestaHWInterface(ros::NodeHandle& nh);
        ~ProchestaHWInterface();
        void init();
        void update(const ros::TimerEvent& e);
        void read();
        void write(ros::Duration elapsed_time);
        ros::ServiceClient client;
        ros::Publisher pub;
        ros::Subscriber sub;
        prochesta_arm_motion::SixJoints joints_pub;
        prochesta_arm_motion::JointSrv joints_read;
    protected:
        hardware_interface::JointStateInterface _joint_state_interface;
        hardware_interface::PositionJointInterface _position_joint_interface;

        // joint_limits_interface::PositionJointSaturationInterface _position_joint_saturation_interface;
        // joint_limits_interface::PositionJointSoftLimitsInterface _position_joint_soft_limits_interface;

        int _num_joints;
        std::string _joint_names[6];
        double _joint_positions[6];
        double _joint_velocities[6];
        double _joint_efforts[6];
        double _joint_position_commands[6];

        ros::NodeHandle _nh;
        ros::Timer _non_realtime_loop;
        ros::Duration _elapsed_time;
        double _loop_hz;
        boost::shared_ptr<controller_manager::ControllerManager> _controller_manager;
};