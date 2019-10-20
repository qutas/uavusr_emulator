#pragma once

#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/OccupancyGrid.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>

#include <dynamic_reconfigure/server.h>
#include <uavusr_emulator/EmulatorParamsConfig.h>

#include <eigen3/Eigen/Dense>
#include <string>

namespace uavusr_emulator {

const std::vector<std::string> mode_names = {
	"UNSET",
	"MANUAL",
	"ACRO",
	"ALTCTL",
	"POSCTL",
	"OFFBOARD",
	"STABILIZED",
	"RATTITUDE",
	"AUTO_MISSION",
	"AUTO_LOITER",
	"AUTO_RTL",
	"AUTO_LAND",
	"AUTO_RTGS",
	"AUTO_READY",
	"AUTO_TAKEOFF",
};

const std::vector<int> airframe_num_motors = {
	4,
	6,
	8
};

class UAVUSREmulator {

	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;

		ros::Publisher pub_pose_;
		ros::Publisher pub_odom_;
		ros::Publisher pub_battery_;
		ros::Publisher pub_state_;
		ros::Publisher pub_grid_rand_;
		ros::Publisher pub_grid_real_;
		ros::Publisher pub_drop_red_;
		ros::Publisher pub_drop_blue_;
		image_transport::Publisher pub_image_;

		//ros::Subscriber sub_goal_;
		ros::Subscriber sub_attitude_;
		ros::Subscriber sub_drop_red_;
		ros::Subscriber sub_drop_blue_;

		ros::Timer timer_pose_;
		ros::Timer timer_state_;
		ros::Timer timer_image_;

		std::string param_frame_id_;
		std::string param_model_id_;
		double param_rate_pose_;
		double param_rate_image_;
		//double param_vel_max_;
		//double param_pos_p_;
		bool param_system_armed_;
		std::string param_system_mode_;
		int param_num_motors_;

		double param_mass_;
		double param_thrust_single_;
		double param_ye_kp_;
		//double param_att_p_;
		dynamic_reconfigure::Server<uavusr_emulator::EmulatorParamsConfig> dyncfg_settings_;

		mavros_msgs::AttitudeTarget attitude_goal_;

		//State Variables
		Eigen::Vector3d p_;
		Eigen::Vector3d v_;
		Eigen::Quaterniond q_;

		image_transport::ImageTransport it_;
		int img_seq_;
		sensor_msgs::Image img_out_w_;
		sensor_msgs::Image img_out_r_;
		sensor_msgs::Image img_out_g_;
		sensor_msgs::Image img_out_b_;

		tf2_ros::TransformBroadcaster tfbr_;

	public:
		UAVUSREmulator( void );

		~UAVUSREmulator( void );

	private:
		void callback_cfg_settings( uavusr_emulator::EmulatorParamsConfig &config, uint32_t level );

		void callback_pose(const ros::TimerEvent& e);
		void callback_state(const ros::TimerEvent& e);
		void callback_image(const ros::TimerEvent& e);

		void callback_attitude(const mavros_msgs::AttitudeTarget::ConstPtr& msg_in);
		void callback_drop_red(const std_msgs::Empty::ConstPtr& msg_in);
		void callback_drop_blue(const std_msgs::Empty::ConstPtr& msg_in);

		double yaw_error_shortest_path(const double y_sp, const double y);

		geometry_msgs::Pose calcHitPoint();
        nav_msgs::OccupancyGrid generateGridData( bool gen_random );
		void generateImageData(sensor_msgs::Image &img, int w, int h, int r, int g, int b);

		double clamp(double x, double min, double max);
};

};
