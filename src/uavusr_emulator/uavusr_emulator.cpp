#include <ros/ros.h>

#include <uavusr_emulator/uavusr_emulator.h>

#include <std_msgs/Empty.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <image_transport/image_transport.h>

#include <string>
#include <math.h>
#include <algorithm>

#define GRAV 9.80665

UAVUSREmulator::UAVUSREmulator() :
	nhp_("~"),
	param_frame_id_("map"),
	param_model_id_("mantis_uav"),
	param_rate_pose_(50.0),
	param_rate_image_(1.0),
	param_vel_max_(1.0),
	it_(nhp_),
	img_seq_(0) {

	//Parameters
	nhp_.param("frame_id", param_frame_id_, param_frame_id_);
	nhp_.param("model_id", param_model_id_, param_model_id_);
	nhp_.param("update_rate_pose", param_rate_pose_, param_rate_pose_);
	nhp_.param("update_rate_image", param_rate_image_, param_rate_image_);
	nhp_.param("vel_max", param_vel_max_, param_vel_max_);

	//Publishers
	pub_pose_ = nhp_.advertise<geometry_msgs::PoseStamped>( param_model_id_ + "/pose", 100 );
	pub_twist_ = nhp_.advertise<geometry_msgs::TwistStamped>( param_model_id_ + "/twist", 100 );
	pub_drop_red_ = nhp_.advertise<geometry_msgs::PoseStamped>( "drop/red/pose", 100 );
	pub_drop_blue_ = nhp_.advertise<geometry_msgs::PoseStamped>( "drop/blue/pose", 100 );

	pub_grid_rand_ = nhp_.advertise<nav_msgs::OccupancyGrid>( "grid/random", 1, true );
	pub_grid_real_ = nhp_.advertise<nav_msgs::OccupancyGrid>( "grid/real", 1, true );

	pub_image_ = it_.advertise("image", 1);

	//Subscribers
	sub_goal_ = nhp_.subscribe<geometry_msgs::PoseStamped>( param_model_id_ + "/goal", 10, &UAVUSREmulator::callback_goal, this );

	sub_drop_red_ = nhp_.subscribe<std_msgs::Empty>( "drop/red", 10, &UAVUSREmulator::callback_drop_red, this );
	sub_drop_blue_ = nhp_.subscribe<std_msgs::Empty>( "drop/blue", 10, &UAVUSREmulator::callback_drop_blue, this );

	//Pregenerate some of the data
	//Images
	generateImageData(img_out_w_, 640, 480, 0xFF, 0xFF, 0xFF);
	generateImageData(img_out_r_, 640, 480, 0xFF, 0x00, 0x00);
	generateImageData(img_out_g_, 640, 480, 0x00, 0xFF, 0x00);
	generateImageData(img_out_b_, 640, 480, 0x00, 0x00, 0xFF);

	//Occupancy Grid
	std::srand( std::time(0) );
	ros::Time grid_stamp = ros::Time::now();
	nav_msgs::OccupancyGrid grid_test_out;
	nav_msgs::OccupancyGrid grid_rand_out;
	grid_test_out.header.frame_id = param_frame_id_;
	grid_test_out.header.stamp = grid_stamp;
	grid_test_out.info.map_load_time = grid_stamp;
	grid_test_out.info.resolution = 0.1;
	grid_test_out.info.width = 50;
	grid_test_out.info.height = 50;
	grid_test_out.info.origin.position.x = -2.5;
	grid_test_out.info.origin.position.y = -2.5;
	grid_test_out.info.origin.orientation.w = 1.0;
	grid_rand_out = grid_test_out;

	int g_obs_x = 20;
	int g_obs_y = 30;
	int g_obs_s = 5;

	for(int gv_x = 0; gv_x < grid_test_out.info.width; gv_x++) {
		for(int gv_y = 0; gv_y < grid_test_out.info.height; gv_y++) {
			//Generate boundary and obstacle
			int8_t g_val = 0;

			if( (gv_x == 0) ||
				(gv_y == 0) ||
				(gv_x == grid_test_out.info.width - 1) ||
				(gv_y == grid_test_out.info.height - 1) ) {
				g_val = 100;
			}

			if( ( ( gv_x >= g_obs_x - g_obs_s) &&
				  ( gv_x <= g_obs_x + g_obs_s) ) &&
				( ( gv_y >= g_obs_y - g_obs_s) &&
				  ( gv_y <= g_obs_y + g_obs_s) ) ) {
				g_val = 100;
			}

			grid_test_out.data.push_back(g_val);

			//Generate random data for the random map
			int8_t rand_val = 100;

			if( std::rand() % 8 ) {
				rand_val = 0;
			}

			grid_rand_out.data.push_back(rand_val);
		}
	}

	pub_grid_rand_.publish(grid_rand_out);
	pub_grid_real_.publish(grid_test_out);

	//Pose
	pose_current_.header.frame_id = param_frame_id_;
	twist_current_.header.frame_id = param_frame_id_;
	pose_goal_.header.frame_id = param_frame_id_;

	pose_current_.pose.position.x = 0.0;
	pose_current_.pose.position.y = 0.0;
	pose_current_.pose.position.z = 0.0;
	pose_goal_.pose.position.x = 0.0;
	pose_goal_.pose.position.y = 0.0;
	pose_goal_.pose.position.z = 1.5;
	pose_current_.pose.orientation.x = 0.0;
	pose_current_.pose.orientation.y = 0.0;
	pose_current_.pose.orientation.z = 0.0;
	pose_current_.pose.orientation.w = 1.0;
	pose_goal_.pose.orientation = pose_current_.pose.orientation;

	twist_current_.twist.linear.x = 0.0;
	twist_current_.twist.linear.y = 0.0;
	twist_current_.twist.linear.z = 0.0;
	twist_current_.twist.angular.x = 0.0;
	twist_current_.twist.angular.y = 0.0;
	twist_current_.twist.angular.z = 0.0;

	//Start the control loop
	timer_pose_ = nhp_.createTimer(ros::Duration(1.0/param_rate_pose_), &UAVUSREmulator::callback_pose, this );
	timer_image_ = nhp_.createTimer(ros::Duration(1.0/param_rate_image_), &UAVUSREmulator::callback_image, this );

	ROS_INFO("UAVUSR Emulator started!");
}

UAVUSREmulator::~UAVUSREmulator() {
}

void UAVUSREmulator::callback_pose(const ros::TimerEvent& e) {
	double dt = 1.0/param_rate_pose_;

	pose_current_.header.stamp = e.current_real;
	twist_current_.header.stamp = e.current_real;

	//TODO: Should be a proper vector to ensure speed was clipped circularly
	//TODO: Add a slight amount of noise
	double vx = -clamp(pose_current_.pose.position.x - pose_goal_.pose.position.x, -param_vel_max_, param_vel_max_);
	double vy = -clamp(pose_current_.pose.position.y - pose_goal_.pose.position.y, -param_vel_max_, param_vel_max_);
	double vz = -clamp(pose_current_.pose.position.z - pose_goal_.pose.position.z, -param_vel_max_, param_vel_max_);

	pose_current_.pose.position.x += vx*dt;
	pose_current_.pose.position.y += vy*dt;
	pose_current_.pose.position.z += vz*dt;

	//TODO: Rotation
	pose_current_.pose.orientation.x = 0.0;
	pose_current_.pose.orientation.y = 0.0;
	pose_current_.pose.orientation.z = 0.0;
	pose_current_.pose.orientation.w = 1.0;

	twist_current_.twist.linear.x = vx;
	twist_current_.twist.linear.y = vy;
	twist_current_.twist.linear.z = vz;

	pub_pose_.publish(pose_current_);
	pub_twist_.publish(twist_current_);
}

void UAVUSREmulator::callback_image(const ros::TimerEvent& e) {
	switch(img_seq_) {
		case 0: {
			img_out_w_.header.stamp = ros::Time::now();
			pub_image_.publish(img_out_w_);

			img_seq_ = 1;
			break;
		}
		case 1: {
			img_out_r_.header.stamp = ros::Time::now();
			pub_image_.publish(img_out_r_);

			img_seq_ = 2;
			break;
		}
		case 2: {
			img_out_g_.header.stamp = ros::Time::now();
			pub_image_.publish(img_out_g_);

			img_seq_ = 3;
			break;
		}
		case 3: {
			img_out_b_.header.stamp = ros::Time::now();
			pub_image_.publish(img_out_b_);

			img_seq_ = 0;
			break;
		}
		default: {
			img_seq_ = 0;
		}
	}
}

void UAVUSREmulator::callback_goal(const geometry_msgs::PoseStamped::ConstPtr& msg_in) {
	pose_goal_ =*msg_in;
}

void UAVUSREmulator::callback_drop_red(const std_msgs::Empty::ConstPtr& msg_in) {
	ROS_INFO("Red servo actuated");

	geometry_msgs::PoseStamped hit;
	hit.header.frame_id = param_frame_id_;
	hit.header.stamp = ros::Time::now();

	hit.pose = calcHitPoint(pose_current_.pose, twist_current_.twist);

	pub_drop_red_.publish(hit);
}

void UAVUSREmulator::callback_drop_blue(const std_msgs::Empty::ConstPtr& msg_in) {
	ROS_INFO("Blue servo actuated");

	geometry_msgs::PoseStamped hit;
	hit.header.frame_id = param_frame_id_;
	hit.header.stamp = ros::Time::now();

	hit.pose = calcHitPoint(pose_current_.pose, twist_current_.twist);

	pub_drop_blue_.publish(hit);
}

geometry_msgs::Pose UAVUSREmulator::calcHitPoint(const geometry_msgs::Pose &p, const geometry_msgs::Twist &v) {
	geometry_msgs::Pose hit;

	//If the simulated UAV is above the ground
	if(p.position.z > 0.0) {
		//Fall time calculation
		// z = vz*t - 0.5*g*t^2
		double a = 0.5*GRAV;
		double b = v.linear.z;
		double c = -p.position.z;
		double t = (-b + std::sqrt(b*b - 4*a*c)) / (2*a);

		//ROS_INFO("Drop: %0.4f, %0.4f, %0.4f", a, b, c);
		//ROS_INFO("Hit Time: %0.4f", t);

		hit.position.x = p.position.x + (v.linear.x * t);
		hit.position.y = p.position.y + (v.linear.y * t);
		hit.position.z = 0.0;
	} else {
		hit.position.x = p.position.x;
		hit.position.y = p.position.y;
		hit.position.z = 0.0;
	}

	hit.orientation.x = 0.0;
	hit.orientation.y = 0.0;
	hit.orientation.z = 0.0;
	hit.orientation.w = 1.0;

	return hit;
}

void UAVUSREmulator::generateImageData(sensor_msgs::Image &img, int w, int h, int r, int g, int b) {
	img.header.frame_id = "camera";
	img.height = h;
	img.width = w;
	img.encoding = "bgr8";
	img.is_bigendian = 0x00;
	img.step = 3 * img.width;

	uint16_t c = 0xFF;
	bool flip = false;
	int ucount = 1;
	int vcount = 1;
	int img_data_counter = 0;

	size_t st0 = (img.step * img.height);
	img.data.resize(st0);

	for(int v = 0; v < img.height; v++) {
		for(int u = 0; u < img.width; u++) {
			if( ucount < u ) {
				ucount = 2*ucount;

				if( flip ) {
					flip = false;
				} else {
					flip = true;
				}
			}

			if( flip ) {
				img.data[img_data_counter] = b;
				img.data[img_data_counter + 1] = g;
				img.data[img_data_counter + 2] = r;
			} else {
				img.data[img_data_counter] = 0x00;
				img.data[img_data_counter + 1] = 0x00;
				img.data[img_data_counter + 2] = 0x00;
			}

			img_data_counter += 3;
		}

		ucount = 1;

		if( vcount < v ) {
			vcount = 2*vcount;

			if( flip ) {
				flip = false;
			} else {
				flip = true;
			}
		}
	}
}

double UAVUSREmulator::clamp(double x, double min, double max) {
    return (x < min) ? min : ( (x > max) ? max : x );
}
