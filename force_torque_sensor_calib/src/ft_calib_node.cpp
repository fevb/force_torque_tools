/*
 *  ft_calib_node.cpp
 *
 *
 *  Created on: Sep 26, 2012
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2012, Francisco Viña, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <kdl_conversions/kdl_msg.h>
#include <kdl_wrapper/kdl_wrapper.h>
#include <sensor_msgs/Imu.h>
#include <moveit/move_group_interface/move_group.h>
#include <force_torque_sensor_calib/ft_calib.h>
#include <eigen3/Eigen/Core>

using namespace Calibration;


class FTCalibNode
{


public:
	ros::NodeHandle n_;
	ros::AsyncSpinner *spinner;
	ros::Subscriber topicSub_ft_raw_;
	ros::Subscriber topicSub_Accelerometer_;
	ros::Subscriber topicSub_JointStates_;

	FTCalibNode()
	{
		n_ = ros::NodeHandle("~");
		spinner = new ros::AsyncSpinner(1);
		spinner->start();

		topicSub_ft_raw_ = n_.subscribe("ft_raw", 1, &FTCalibNode::topicCallback_ft_raw, this);
		topicSub_Accelerometer_ = n_.subscribe("imu", 1, &FTCalibNode::topicCallback_imu, this);
		topicSub_JointStates_ = n_.subscribe("joint_states", 1, &FTCalibNode::topicCallback_joint_states, this);

        m_pose_counter = 0;

		m_received_ft = false;
		m_received_imu = false;

		m_finished = false;
		m_tf_listener = new tf::TransformListener();

		m_ft_calib = new FTCalib();
	}

	~FTCalibNode()
	{
		saveCalibData();
		delete spinner;
		delete m_group;
		delete m_ft_calib;
		delete m_tf_listener;
	}


	bool getROSParameters()
	{

        // Get joint names
        XmlRpc::XmlRpcValue JointNamesXmlRpc;
		if (n_.hasParam("joint_names"))
		{
			n_.getParam("joint_names", JointNamesXmlRpc);
		}

		else
		{
			ROS_ERROR("Parameter joint_names not set, shutting down node...");
			n_.shutdown();
			return false;
		}


		/// Resize and assign of values to the JointNames
        m_joint_names.resize(JointNamesXmlRpc.size());
		for (int i = 0; i < JointNamesXmlRpc.size(); i++)
		{
            m_joint_names[i] = (std::string)JointNamesXmlRpc[i];
		}

        m_DOF = m_joint_names.size();
        m_joint_states.joint_names = m_joint_names;
		m_joint_states.actual.positions.resize(m_DOF);
		m_joint_states.actual.velocities.resize(m_DOF);
        m_joint_states.actual.accelerations.resize(m_DOF);

        for(unsigned int i=0; i<m_DOF; i++)
        {
            m_joint_states.actual.positions[i] = 0.0;
            m_joint_states.actual.velocities[i] = 0.0;
            m_joint_states.actual.accelerations[i] = 0.0;
        }

        // Get the base link for robot kinematics calculations
        if(n_.hasParam("base_link"))
        {
            n_.getParam("base_link", m_base_link);
        }

        else
        {
            ROS_ERROR("Parameter base_link not set, shutting down node...");
            n_.shutdown();
            return false;
        }

        // Get the force-torque sensor link for robot kinematics calculations
        if(n_.hasParam("ft_sensor_link"))
        {
            n_.getParam("ft_sensor_link", m_ft_sensor_link);
        }

        else
        {
            ROS_ERROR("Parameter ft_sensor_link not set, shutting down node...");
            n_.shutdown();
            return false;
        }

		// Get the moveit group name
		if(n_.hasParam("moveit_group_name"))
		{
			n_.getParam("moveit_group_name", m_moveit_group_name);
		}

		else
		{
			ROS_ERROR("No moveit_group_name parameter, shutting down node...");
			n_.shutdown();
			return false;
		}

		// Get the name of output calibration file
		if(n_.hasParam("calib_file_name"))
		{
			n_.getParam("calib_file_name", m_calib_file_name);
		}

		else
		{
			ROS_WARN("No calib_file_name parameter, setting to default 'ft_calib.yaml'");
			m_calib_file_name = std::string("ft_calib_data.yaml");
		}


		// Get the name of calibration file directory
		if(n_.hasParam("calib_file_dir"))
		{
			n_.getParam("calib_file_dir", m_calib_file_dir);
		}

		else
		{
			ROS_WARN("No calib_file_dir parameter, setting to default '~/.ros/ft_calib' ");
			m_calib_file_dir = std::string("~/.ros/ft_calib");
		}

		if(n_.hasParam("poses_frame_id"))
		{
			n_.getParam("poses_frame_id", m_poses_frame_id);
		}

		else
		{
			ROS_ERROR("No poses_frame_id parameter, shutting down node ...");
			n_.shutdown();
			return false;
		}

		// whether the user wants to use random poses
		n_.param("random_poses", m_random_poses, false);

		// number of random poses
		n_.param("number_random_poses", m_number_random_poses, 30);

		return true;
	}

	// connects to the move arm servers
    void init()
	{
		m_group = new move_group_interface::MoveGroup(m_moveit_group_name);
        if (!m_kdl_wrapper.init(m_base_link, m_ft_sensor_link))
        {
            ROS_ERROR("Couldn't initialize KDL wrapper ... shutting down node...");
            n_.shutdown();
            return;
        }
	}


	// Calibrates the FT sensor by putting the arm in several different positions
	bool moveNextPose()
	{

		std::stringstream ss;
		ss << m_pose_counter;
		Eigen::Matrix<double, 6, 1> pose;

		// either find poses from the parameter server
		// poses should be in "pose%d" format (e.g. pose0, pose1, pose2 ...)
		// and they should be float arrays of size 6
		if(!m_random_poses)
		{
			if(!getPose("pose"+ss.str(), pose))
			{
				ROS_INFO("Finished group %s poses", m_group->getName().c_str());
				m_finished = true;
				return true;
			}

			geometry_msgs::Pose pose_;
            pose_.position.x = pose(0);
			pose_.position.y = pose(1);
			pose_.position.z = pose(2);

			tf::Quaternion q;
			q.setRPY((double)pose(3), (double)pose(4), (double)pose(5));

			tf::quaternionTFToMsg(q, pose_.orientation);

			geometry_msgs::PoseStamped pose_stamped;
			pose_stamped.pose = pose_;
			pose_stamped.header.frame_id = m_poses_frame_id;
			pose_stamped.header.stamp = ros::Time::now();

			m_group->setPoseTarget(pose_stamped);

		}
		else // or execute random poses
		{
			if(m_pose_counter<m_number_random_poses)
			{
				m_group->setRandomTarget();
				ROS_INFO("Executing pose %d",m_pose_counter);
			}

			else
			{
				ROS_INFO("Finished group %s random poses", m_group->getName().c_str());
				m_finished = true;
				return true;
			}
		}


		m_pose_counter++;
		m_group->move();
		ROS_INFO("Finished executing pose %d", m_pose_counter-1);
		return true;
	}

	// gets the next pose from the parameter server
	// pose in [x y z r p y] format ([m], [rad])
	bool getPose(const std::string &pose_param_name, Eigen::Matrix<double, 6, 1> &pose)
	{
		XmlRpc::XmlRpcValue PoseXmlRpc;
		if(n_.hasParam(pose_param_name))
		{
			n_.getParam(pose_param_name, PoseXmlRpc);
		}

		else
		{
			ROS_WARN("Pose parameter %s not found", pose_param_name.c_str());
			return false;
		}

		if(PoseXmlRpc.size()!=6)
		{
			ROS_ERROR("Pose parameter %s wrong size (must be 6)", pose_param_name.c_str());
			return false;
		}

		for(unsigned int i=0; i<6; i++)
			pose(i) = (double)PoseXmlRpc[i];

		return true;
	}

	// prints out the pose (3-D positions) of the calibration frame at each of the positions
	// of the left arm
	void saveCalibData()
	{
		double mass;
		Eigen::Vector3d COM_pos;
		Eigen::Vector3d f_bias;
		Eigen::Vector3d t_bias;

        getCalibParams(mass, COM_pos, f_bias, t_bias);

		XmlRpc::XmlRpcValue bias;
		bias.setSize(6);
		for(unsigned int i=0; i<3; i++)
			bias[i] = (double)f_bias(i);

		for(unsigned int i=0; i<3; i++)
			bias[i+3] = (double)t_bias(i);

		XmlRpc::XmlRpcValue COM_pose;
		COM_pose.setSize(6);
		for(unsigned int i=0; i<3; i++)
			COM_pose[i] = (double)COM_pos(i);

		for(unsigned int i=0; i<3; i++)
			COM_pose[i+3] = 0.0;

		// set the parameters in the parameter server
		n_.setParam("/ft_calib/bias", bias);
		n_.setParam("/ft_calib/gripper_mass", mass);
		n_.setParam("/ft_calib/gripper_com_frame_id", m_ft_raw.header.frame_id.c_str());
		n_.setParam("/ft_calib/gripper_com_pose", COM_pose);

		// dump the parameters to YAML file
		std::string file = m_calib_file_dir + std::string("/") + m_calib_file_name;

		// first create the directory
		std::string command = std::string("mkdir -p ") + m_calib_file_dir;
		std::system(command.c_str());

		// now dump the yaml file
		command.clear();
		command = std::string("rosparam dump ") + file + std::string(" /ft_calib");
		std::system(command.c_str());
	}

	// finished moving the arm through the poses set in the config file
	bool finished()
	{
		return(m_finished);
	}

	void topicCallback_ft_raw(const geometry_msgs::WrenchStamped::ConstPtr &msg)
	{
        ROS_DEBUG("In ft sensor callback");
		m_ft_raw = *msg;
		m_received_ft = true;
	}


	// gets readings from accelerometer and transforms them to the FT sensor frame
	void topicCallback_imu(const sensor_msgs::Imu::ConstPtr &msg)
	{
		ROS_DEBUG("In accelerometer read callback");

		m_imu= *msg;
		m_received_imu = true;
	}

    void topicCallback_joint_states(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg)
	{
        ROS_DEBUG("In joint states callback");
        m_joint_states = *msg;
	}

    void calibrate()
	{

		if(!m_received_ft)
		{
			ROS_ERROR("Haven't received F/T sensor measurements");
			return;
		}

		if(!m_received_imu)
		{
			ROS_ERROR("Haven't received accelerometer readings");
			return;
		}

		// express gravity vector in F/T sensor frame
        geometry_msgs::Vector3Stamped g;
        g.header.stamp = ros::Time();
        g.header.frame_id = m_imu.header.frame_id;
        g.vector = m_imu.linear_acceleration;

		geometry_msgs::Vector3Stamped gravity_ft_frame;

		try
		{
            m_tf_listener->transformVector(m_ft_raw.header.frame_id, g, gravity_ft_frame);
		}

		catch(tf::TransformException &ex)
		{
			ROS_ERROR("Error transforming accelerometer reading to the F/T sensor frame");
			ROS_ERROR("%s.", ex.what());
			return;
		}

        KDL::Vector gravity(gravity_ft_frame.vector.x,
                            gravity_ft_frame.vector.y,
                            gravity_ft_frame.vector.z);

        // calculate the acceleration of the FT sensor frame
        KDL::JntArrayAcc q;
        q.resize(m_DOF);

        for(unsigned int i=0; i<m_DOF; i++)
        {
            q.q(i) = m_joint_states.actual.positions[i];
            q.qdot(i) = m_joint_states.actual.velocities[i];
            q.qdotdot(i) = m_joint_states.actual.accelerations[i];
        }


        KDL::FrameAcc ft_sensor_acc_base_frame;
        m_kdl_wrapper.fk_solver_acc->JntToCart(q, ft_sensor_acc_base_frame);

        // transform acceleration so that it is expressed in the ft sensor frame
        KDL::Frame F_ft_sensor;
        m_kdl_wrapper.fk_solver_pos->JntToCart(q.q, F_ft_sensor);

        KDL::FrameAcc ft_sensor_acc;
        KDL::Rotation R_ft_sensor_base = ft_sensor_acc_base_frame.M.R.Inverse();

        ft_sensor_acc.M.w = R_ft_sensor_base*ft_sensor_acc_base_frame.M.w;
        ft_sensor_acc.M.dw = R_ft_sensor_base*ft_sensor_acc_base_frame.M.dw;
        ft_sensor_acc.p.dv = R_ft_sensor_base*ft_sensor_acc_base_frame.p.dv;

        KDL::Wrench ft_raw_meas;
        tf::wrenchMsgToKDL(m_ft_raw.wrench, ft_raw_meas);

        ft_raw_meas = -1.0*ft_raw_meas;

        m_ft_sensor_params = m_ft_calib->calibrate(ft_sensor_acc, gravity, ft_raw_meas);
    }

    void getCalibParams(double &mass, Eigen::Vector3d &COM_pos, Eigen::Vector3d &f_bias, Eigen::Vector3d &t_bias)
	{


        mass = m_ft_sensor_params(0,0);
		if(mass<=0.0)
		{
			ROS_ERROR("Error in estimated mass (<= 0)");
			//		return;
		}

        Eigen::Vector3d center_mass_position(m_ft_sensor_params(1,0)/mass,
                m_ft_sensor_params(2,0)/mass,
                m_ft_sensor_params(3,0)/mass);

		COM_pos = center_mass_position;

        f_bias(0) = -m_ft_sensor_params(4,0);
        f_bias(1) = -m_ft_sensor_params(5,0);
        f_bias(2) = -m_ft_sensor_params(6,0);
        t_bias(0) = -m_ft_sensor_params(7,0);
        t_bias(1) = -m_ft_sensor_params(8,0);
        t_bias(2) = -m_ft_sensor_params(9,0);

	}

private:

	move_group_interface::MoveGroup *m_group;

	unsigned int m_DOF;
	std::vector<std::string> m_joint_names;
	control_msgs::JointTrajectoryControllerState m_joint_states;

    std::string m_base_link;
    std::string m_ft_sensor_link;
    KDLWrapper m_kdl_wrapper;

    unsigned int m_pose_counter;

	bool m_finished;

	bool m_received_ft;
	bool m_received_imu;

	// ft calib stuff
	FTCalib *m_ft_calib;

    // estimated FT sensor parameters
    Eigen::Matrix<double, 10, 1> m_ft_sensor_params;

	// expressed in FT sensor frame
    geometry_msgs::WrenchStamped m_ft_raw;

	// accelerometer readings
	sensor_msgs::Imu m_imu;

	tf::TransformListener *m_tf_listener;

	//	***** ROS parameters ***** //
	// name of the moveit group
	std::string m_moveit_group_name;

	// name of output calibration file
	std::string m_calib_file_name;

	// name of output directory
	// default: ~/.ros/ft_calib
	std::string m_calib_file_dir;

	// frame id of the poses to be executed
	std::string m_poses_frame_id;

	// if the user wants to execute just random poses
	// default: false
	bool m_random_poses;

	// number of random poses
	// default: 30
	int m_number_random_poses;

};

int main(int argc, char **argv)
{
	ros::init (argc, argv, "ft_calib_node");
	ros::NodeHandle nh;

	FTCalibNode ft_calib_node;
	if(!ft_calib_node.getROSParameters())
	{
		ft_calib_node.n_.shutdown();
		ROS_ERROR("Error getting ROS parameters");

	}

	ft_calib_node.init();

	/// main loop
	double loop_rate_;
	ft_calib_node.n_.param("loop_rate", loop_rate_, 650.0);
	ros::Rate loop_rate(loop_rate_); // Hz

	// waiting time after end of each pose to take F/T measurements
	double wait_time;
	ft_calib_node.n_.param("wait_time", wait_time, 4.0);

    bool ret = false;
    unsigned int n_measurements = 0;

	ros::Time t_end_move_arm = ros::Time::now();

	while (ft_calib_node.n_.ok() && !ft_calib_node.finished())
	{

		//		Move the arm, then calibrate sensor
		if(!ret)
		{
			ret = ft_calib_node.moveNextPose();
			t_end_move_arm = ros::Time::now();
		}

        // take 100 measurements at each position to calibrate the sensor
		else if ((ros::Time::now() - t_end_move_arm).toSec() > wait_time)
		{
            while(n_measurements<100)
            {
                ft_calib_node.calibrate();
                ros::spinOnce();
                loop_rate.sleep();
                n_measurements++;
            }

            ret = false;
            n_measurements = 0;

            double mass;
            Eigen::Vector3d COM_pos;
            Eigen::Vector3d f_bias;
            Eigen::Vector3d t_bias;

            ft_calib_node.getCalibParams(mass, COM_pos, f_bias, t_bias);
            std::cout << "-------------------------------------------------------------" << std::endl;
            std::cout << "Current calibration estimate:" << std::endl;
            std::cout << std::endl << std::endl;

            std::cout << "Mass: " << mass << std::endl << std::endl;

            std::cout << "Center of mass position (relative to FT sensor frame):" << std::endl;
            std::cout << "[" << COM_pos(0) << ", " << COM_pos(1) << ", " << COM_pos(2) << "]";
            std::cout << std::endl << std::endl;


            std::cout << "FT bias: " << std::endl;
            std::cout << "[" << f_bias(0) << ", " << f_bias(1) << ", " << f_bias(2) << ", ";
            std::cout << t_bias(0) << ", " << t_bias(1) << ", " << t_bias(2) << "]";
            std::cout << std::endl << std::endl;


            std::cout << "-------------------------------------------------------------" << std::endl << std::endl << std::endl;
            ft_calib_node.saveCalibData();

		}


		ros::spinOnce();
		loop_rate.sleep();
	}

	ft_calib_node.saveCalibData();
	ros::shutdown();
	return 0;
}
