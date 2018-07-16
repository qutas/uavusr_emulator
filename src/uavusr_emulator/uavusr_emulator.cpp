#include <ros/ros.h>

#include <uavusr_emulator/uavusr_emulator.h>

#include <std_msgs/Empty.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <image_transport/image_transport.h>

#include <string>
#include <math.h>
#include <algorithm>
#include <cstdlib>
#include <time.h>

#include <eigen3/Eigen/Dense>

#define GRAV 9.80665

UAVUSREmulator::UAVUSREmulator() :
	nh_(),
	nhp_("~"),
	param_frame_id_("map"),
	param_model_id_("uav"),
	param_rate_pose_(50.0),
	param_rate_image_(1.0),
	param_vel_max_(1.0),
	param_pos_p_(1.0),
	it_(nh_),
	img_seq_(0) {

	srand(time(NULL));

	//Parameters
	nhp_.param("frame_id", param_frame_id_, param_frame_id_);
	nhp_.param("model_id", param_model_id_, param_model_id_);
	nhp_.param("update_rate_pose", param_rate_pose_, param_rate_pose_);
	nhp_.param("update_rate_image", param_rate_image_, param_rate_image_);
	nhp_.param("vel_max", param_vel_max_, param_vel_max_);
	nhp_.param("pos_gain", param_pos_p_, param_pos_p_);

	//Publishers
	pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>( "pose", 100 );
	pub_odom_ = nh_.advertise<nav_msgs::Odometry>( "odom", 100 );
	pub_state_ = nh_.advertise<mavros_msgs::State>( "state", 100 );
	pub_battery_ = nh_.advertise<sensor_msgs::BatteryState>( "battery", 100 );
	pub_drop_red_ = nh_.advertise<geometry_msgs::PoseStamped>( "drop/red/pose", 100 );
	pub_drop_blue_ = nh_.advertise<geometry_msgs::PoseStamped>( "drop/blue/pose", 100 );

	pub_grid_rand_ = nh_.advertise<nav_msgs::OccupancyGrid>( "/grid/random", 1, true );
	pub_grid_real_ = nh_.advertise<nav_msgs::OccupancyGrid>( "/grid/real", 1, true );

	pub_image_ = it_.advertise("camera/image", 1);

	//Subscribers
	sub_goal_ = nh_.subscribe<mavros_msgs::PositionTarget>( "reference/triplet", 10, &UAVUSREmulator::callback_goal, this );

	sub_drop_red_ = nh_.subscribe<std_msgs::Empty>( "drop/red", 10, &UAVUSREmulator::callback_drop_red, this );
	sub_drop_blue_ = nh_.subscribe<std_msgs::Empty>( "drop/blue", 10, &UAVUSREmulator::callback_drop_blue, this );

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
	odom_current_.header.frame_id = param_frame_id_;
	pt_goal_.header.frame_id = param_frame_id_;

	odom_current_.pose.pose.position.x = 0.0;
	odom_current_.pose.pose.position.y = 0.0;
	odom_current_.pose.pose.position.z = 0.0;
	odom_current_.pose.pose.orientation.x = 0.0;
	odom_current_.pose.pose.orientation.y = 0.0;
	odom_current_.pose.pose.orientation.z = 0.0;
	odom_current_.pose.pose.orientation.w = 1.0;
	odom_current_.twist.twist.linear.x = 0.0;
	odom_current_.twist.twist.linear.y = 0.0;
	odom_current_.twist.twist.linear.z = 0.0;
	odom_current_.twist.twist.angular.x = 0.0;
	odom_current_.twist.twist.angular.y = 0.0;
	odom_current_.twist.twist.angular.z = 0.0;

	pt_goal_.coordinate_frame = pt_goal_.FRAME_LOCAL_NED;
	pt_goal_.type_mask |= pt_goal_.IGNORE_VX;
	pt_goal_.type_mask |= pt_goal_.IGNORE_VY;
	pt_goal_.type_mask |= pt_goal_.IGNORE_VZ;
	pt_goal_.type_mask |= pt_goal_.IGNORE_AFX;
	pt_goal_.type_mask |= pt_goal_.IGNORE_AFY;
	pt_goal_.type_mask |= pt_goal_.IGNORE_AFZ;
	pt_goal_.type_mask |= pt_goal_.FORCE;
	pt_goal_.type_mask |= pt_goal_.IGNORE_YAW_RATE;
	pt_goal_.position.x = 0.0;
	pt_goal_.position.y = 0.0;
	pt_goal_.position.z = 1.5;
	pt_goal_.velocity.x = 0.0;
	pt_goal_.velocity.y = 0.0;
	pt_goal_.velocity.z = 0.0;
	pt_goal_.yaw = 0.0;
	pt_goal_.yaw_rate = 0.0;

	//Start the control loop
	timer_pose_ = nhp_.createTimer(ros::Duration(1.0/param_rate_pose_), &UAVUSREmulator::callback_pose, this );
	timer_state_ = nhp_.createTimer(ros::Duration(1.0), &UAVUSREmulator::callback_state, this );
	timer_image_ = nhp_.createTimer(ros::Duration(1.0/param_rate_image_), &UAVUSREmulator::callback_image, this );

	ROS_INFO("UAVUSR Emulator started!");
}

UAVUSREmulator::~UAVUSREmulator() {
}

void UAVUSREmulator::callback_pose(const ros::TimerEvent& e) {
	double dt = 1.0/param_rate_pose_;

	odom_current_.header.stamp = e.current_real;
	/*
	double vx = -clamp(, -param_vel_max_, param_vel_max_);
	double vy = -clamp(pose_current_.pose.position.y - pose_goal_.pose.position.y, -param_vel_max_, param_vel_max_);
	double vz = -clamp(pose_current_.pose.position.z - pose_goal_.pose.position.z, -param_vel_max_, param_vel_max_);
	*/
	Eigen::Vector3d vel = param_pos_p_* Eigen::Vector3d(pt_goal_.position.x - odom_current_.pose.pose.position.x,
														pt_goal_.position.y - odom_current_.pose.pose.position.y,
														pt_goal_.position.z - odom_current_.pose.pose.position.z);

	double vscale  = vel.norm() / param_vel_max_;
	if(vscale > 1.0) {
		//Going to move too fast, rescale to slow down
		vel = vel / vscale;
	}

	odom_current_.pose.pose.position.x += vel.x()*dt;
	odom_current_.pose.pose.position.y += vel.y()*dt;
	odom_current_.pose.pose.position.z += vel.z()*dt;

	double turn_rate = 0.8/param_rate_pose_;

	Eigen::Quaterniond q_c = Eigen::Quaterniond(Eigen::AngleAxisd(pt_goal_.yaw, Eigen::Vector3d::UnitZ()));
	Eigen::Quaterniond q_y = Eigen::Quaterniond(odom_current_.pose.pose.orientation.w,
												odom_current_.pose.pose.orientation.x,
												odom_current_.pose.pose.orientation.y,
												odom_current_.pose.pose.orientation.z).normalized().slerp(turn_rate, q_c);

	odom_current_.pose.pose.orientation.x = q_y.x();
	odom_current_.pose.pose.orientation.y = q_y.y();
	odom_current_.pose.pose.orientation.z = q_y.z();
	odom_current_.pose.pose.orientation.w = q_y.w();

	Eigen::Vector3d bv = q_y.toRotationMatrix().inverse()*vel;
	odom_current_.twist.twist.linear.x = bv.x();
	odom_current_.twist.twist.linear.y = bv.y();
	odom_current_.twist.twist.linear.z = bv.z();

	geometry_msgs::PoseStamped msg_out_pose;
	msg_out_pose.header = odom_current_.header;
	msg_out_pose.pose = odom_current_.pose.pose;

	pub_pose_.publish(msg_out_pose);
	pub_odom_.publish(odom_current_);
}

void UAVUSREmulator::callback_state(const ros::TimerEvent& e) {
	mavros_msgs::State msg_out_state;
	sensor_msgs::BatteryState msg_out_battery;

	msg_out_state.header.stamp = e.current_real;
	msg_out_state.header.frame_id = param_frame_id_;
	msg_out_state.connected = true;
	msg_out_state.armed = true;
	msg_out_state.guided = true;
	msg_out_state.mode = "OFFBOARD";
	msg_out_state.system_status = 4;	//XXX: MAV_STATE_ACTIVE

	//random voltages around the 4.0V mark
	double v1 = 0.02*((rand() % 10) - 5) + 4;
	double v2 = 0.02*((rand() % 10) - 5) + 4;
	double v3 = 0.02*((rand() % 10) - 5) + 4;
	msg_out_battery.header.stamp = e.current_real;
	msg_out_battery.header.frame_id = param_frame_id_;
	msg_out_battery.voltage = v1 + v2 + v3;
	msg_out_battery.current = NAN;
	msg_out_battery.capacity = NAN;
	msg_out_battery.design_capacity = 4.2;
	msg_out_battery.percentage = (msg_out_battery.voltage - 11.1) / (1.5);
	msg_out_battery.power_supply_status = msg_out_battery.POWER_SUPPLY_STATUS_DISCHARGING;
	msg_out_battery.power_supply_health = msg_out_battery.POWER_SUPPLY_HEALTH_GOOD;
	msg_out_battery.power_supply_technology = msg_out_battery.POWER_SUPPLY_TECHNOLOGY_LIPO;
	msg_out_battery.cell_voltage.push_back(v1);
	msg_out_battery.cell_voltage.push_back(v2);
	msg_out_battery.cell_voltage.push_back(v3);
	msg_out_battery.location = "primary";
	msg_out_battery.location = "DEADBEEF";

	pub_state_.publish(msg_out_state);
	pub_battery_.publish(msg_out_battery);
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

void UAVUSREmulator::callback_goal(const mavros_msgs::PositionTarget::ConstPtr& msg_in) {
	pt_goal_ =*msg_in;
}

void UAVUSREmulator::callback_drop_red(const std_msgs::Empty::ConstPtr& msg_in) {
	ROS_INFO("Red servo actuated");

	geometry_msgs::PoseStamped hit;
	hit.header.frame_id = param_frame_id_;
	hit.header.stamp = ros::Time::now();

	hit.pose = calcHitPoint();

	pub_drop_red_.publish(hit);
}

void UAVUSREmulator::callback_drop_blue(const std_msgs::Empty::ConstPtr& msg_in) {
	ROS_INFO("Blue servo actuated");

	geometry_msgs::PoseStamped hit;
	hit.header.frame_id = param_frame_id_;
	hit.header.stamp = ros::Time::now();

	hit.pose = calcHitPoint();

	pub_drop_blue_.publish(hit);
}

geometry_msgs::Pose UAVUSREmulator::calcHitPoint() {
	geometry_msgs::Pose hit;
	Eigen::Vector3d p(odom_current_.pose.pose.position.x,
					  odom_current_.pose.pose.position.y,
					  odom_current_.pose.pose.position.z);
	Eigen::Quaterniond q(odom_current_.pose.pose.orientation.w,
						 odom_current_.pose.pose.orientation.x,
						 odom_current_.pose.pose.orientation.y,
						 odom_current_.pose.pose.orientation.z);
	//Linear velocity
	Eigen::Vector3d v = q.toRotationMatrix()*Eigen::Vector3d(odom_current_.twist.twist.linear.x,
															 odom_current_.twist.twist.linear.y,
															 odom_current_.twist.twist.linear.z);

	//If the simulated UAV is above the ground
	if(p.z() > 0.0) {
		//Fall time calculation
		// z = vz*t - 0.5*g*t^2
		double a = 0.5*GRAV;
		double b = v.z();
		double c = -p.z();
		double t = (-b + std::sqrt(b*b - 4*a*c)) / (2*a);

		//ROS_INFO("Drop: %0.4f, %0.4f, %0.4f", a, b, c);
		//ROS_INFO("Hit Time: %0.4f", t);

		hit.position.x = p.x() + (v.x() * t);
		hit.position.y = p.y() + (v.y() * t);
		hit.position.z = 0.0;
	} else {
		hit.position.x = p.x();
		hit.position.y = p.y();
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
