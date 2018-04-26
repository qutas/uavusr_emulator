#pragma once

#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <image_transport/image_transport.h>

#include <string>

class UAVUSREmulator {
	private:
		ros::NodeHandle nhp_;

		ros::Publisher pub_pose_;
		ros::Publisher pub_grid_rand_;
		ros::Publisher pub_grid_real_;
		image_transport::Publisher pub_image_;

		ros::Subscriber sub_goal_;
		ros::Subscriber sub_drop_red_;
		ros::Subscriber sub_drop_blue_;

		ros::Timer timer_pose_;
		ros::Timer timer_image_;

		std::string param_frame_id_;
		std::string param_model_id_;
		double param_rate_pose_;
		double param_rate_image_;
		double param_vel_max_;

		geometry_msgs::PoseStamped pose_goal_;
		geometry_msgs::PoseStamped pose_current_;

		image_transport::ImageTransport it_;
		int img_seq_;
		sensor_msgs::Image img_out_w_;
		sensor_msgs::Image img_out_r_;
		sensor_msgs::Image img_out_g_;
		sensor_msgs::Image img_out_b_;


	public:
		UAVUSREmulator( void );

		~UAVUSREmulator( void );

		void callback_pose(const ros::TimerEvent& e);
		void callback_image(const ros::TimerEvent& e);

		void callback_goal(const geometry_msgs::PoseStamped::ConstPtr& msg_in);
		void callback_drop_red(const std_msgs::Empty::ConstPtr& msg_in);
		void callback_drop_blue(const std_msgs::Empty::ConstPtr& msg_in);

		void generateImageData(sensor_msgs::Image &img, int w, int h, int r, int g, int b);

		double clamp(double x, double min, double max);
};
