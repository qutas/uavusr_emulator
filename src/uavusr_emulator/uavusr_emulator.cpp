#include <ros/ros.h>

#include <uavusr_emulator/uavusr_emulator.h>

#include <std_msgs/Empty.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <image_transport/image_transport.h>

#include <string>
#include <math.h>
#include <algorithm>
#include <cstdlib>
#include <time.h>

#include <eigen3/Eigen/Dense>

#define GRAV 9.80665

namespace uavusr_emulator {

UAVUSREmulator::UAVUSREmulator() :
	nh_(),
	nhp_("~"),
	param_frame_id_("map"),
	param_model_id_("uav"),
	param_rate_pose_(50.0),
	param_rate_image_(1.0),
	//param_mass_(1.0),
	//param_thrust_single_(5.0),
	//param_ye_kp_(0.8),
	dyncfg_settings_(ros::NodeHandle(nhp_)),
	it_(nh_),
	img_seq_(0) {

	srand(time(NULL));

	dyncfg_settings_.setCallback(boost::bind(&UAVUSREmulator::callback_cfg_settings, this, _1, _2));

	//Parameters
	nhp_.param("frame_id", param_frame_id_, param_frame_id_);
	nhp_.param("model_id", param_model_id_, param_model_id_);
	nhp_.param("update_rate_pose", param_rate_pose_, param_rate_pose_);
	nhp_.param("update_rate_image", param_rate_image_, param_rate_image_);

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
	//sub_goal_ = nh_.subscribe<mavros_msgs::PositionTarget>( "reference/triplet", 10, &UAVUSREmulator::callback_goal, this );
	sub_attitude_ = nh_.subscribe<mavros_msgs::AttitudeTarget>( "reference/attitude", 10, &UAVUSREmulator::callback_attitude, this );

	sub_drop_red_ = nh_.subscribe<std_msgs::Empty>( "drop/red", 10, &UAVUSREmulator::callback_drop_red, this );
	sub_drop_blue_ = nh_.subscribe<std_msgs::Empty>( "drop/blue", 10, &UAVUSREmulator::callback_drop_blue, this );

	//Pregenerate some of the data
	//Images
	generateImageData(img_out_w_, 640, 480, 0xFF, 0xFF, 0xFF);
	generateImageData(img_out_r_, 640, 480, 0xFF, 0x00, 0x00);
	generateImageData(img_out_g_, 640, 480, 0x00, 0xFF, 0x00);
	generateImageData(img_out_b_, 640, 480, 0x00, 0x00, 0xFF);

	//Occupancy Grid
	pub_grid_rand_.publish(generateGridData(true));
	pub_grid_real_.publish(generateGridData(false));

	//Pose
	/*
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
	*/

	p_ = Eigen::Vector3d::Zero();
	v_ = Eigen::Vector3d::Zero();
	q_ = Eigen::Quaterniond::Identity();

	attitude_goal_.type_mask = attitude_goal_.IGNORE_ROLL_RATE |
							   attitude_goal_.IGNORE_PITCH_RATE |
							   attitude_goal_.IGNORE_YAW_RATE;
	attitude_goal_.orientation.x = 0.0;
	attitude_goal_.orientation.y = 0.0;
	attitude_goal_.orientation.z = 0.0;
	attitude_goal_.orientation.w = 1.0;
	attitude_goal_.body_rate.x = 0.0;
	attitude_goal_.body_rate.y = 0.0;
	attitude_goal_.body_rate.z = 0.0;
	attitude_goal_.thrust = 0.0;


	//Start the control loop
	timer_pose_ = nhp_.createTimer(ros::Duration(1.0/param_rate_pose_), &UAVUSREmulator::callback_pose, this );
	timer_state_ = nhp_.createTimer(ros::Duration(1.0), &UAVUSREmulator::callback_state, this );
	timer_image_ = nhp_.createTimer(ros::Duration(1.0/param_rate_image_), &UAVUSREmulator::callback_image, this );

	ROS_INFO("UAVUSR Emulator started!");
}

UAVUSREmulator::~UAVUSREmulator() {
}

void UAVUSREmulator::callback_cfg_settings( uavusr_emulator::EmulatorParamsConfig &config, uint32_t level ) {
	param_system_armed_ = config.system_armed;
    param_system_mode_ = uavusr_emulator::mode_names.at(config.mode);
    param_ye_kp_ = config.psi_delay;

    param_mass_ = config.mass;
    param_thrust_single_ = config.thrust_single;
    param_num_motors_ = uavusr_emulator::airframe_num_motors.at(config.airframe);
}

double UAVUSREmulator::yaw_error_shortest_path(const double y_sp, const double y) {
	double ye = y_sp - y;

	while(fabs(ye) > M_PI)
		ye += (ye > 0.0) ? -2*M_PI : 2*M_PI;

	return ye;
}

void UAVUSREmulator::callback_pose(const ros::TimerEvent& e) {
	double dt = 1.0/param_rate_pose_;

	double thrust = 0.0;
	Eigen::Quaterniond q_sp(1.0, 0.0, 0.0, 0.0);

	if( param_system_armed_ && (attitude_goal_.header.stamp > ros::Time(0)) ) {
		//Accept desired vector as current vector
		q_sp = Eigen::Quaterniond(attitude_goal_.orientation.w,
								  attitude_goal_.orientation.x,
								  attitude_goal_.orientation.y,
								  attitude_goal_.orientation.z);
		q_sp.normalize();

		thrust = clamp(attitude_goal_.thrust, 0.0, 1.0);
	} else {
		attitude_goal_.header.stamp = ros::Time(0);
	}

	/*
	double vx = -clamp(, -param_vel_max_, param_vel_max_);
	double vy = -clamp(pose_current_.pose.position.y - pose_goal_.pose.position.y, -param_vel_max_, param_vel_max_);
	double vz = -clamp(pose_current_.pose.position.z - pose_goal_.pose.position.z, -param_vel_max_, param_vel_max_);
	*/
	/*
	Eigen::Vector3d vel = param_pos_p_* Eigen::Vector3d(pt_goal_.position.x - odom_current_.pose.pose.position.x,
														pt_goal_.position.y - odom_current_.pose.pose.position.y,
														pt_goal_.position.z - odom_current_.pose.pose.position.z);


	vel += Eigen::Vector3d(pt_goal_.velocity.x, pt_goal_.velocity.y, pt_goal_.velocity.z);

	double vscale  = vel.norm() / param_vel_max_;
	if(vscale > 1.0) {
		//Going to move too fast, rescale to slow down
		vel = vel / vscale;
	}
	*/

	//TODO:
	//	Calculate acceleration vector
	//	Calculate yaw delta
	//	Propogate linear dynamics
	//	Propogate yaw rotation
	//	Fill in message details

	/*
	odom_current_.pose.pose.position.x += vel.x()*dt;
	odom_current_.pose.pose.position.y += vel.y()*dt;
	odom_current_.pose.pose.position.z += vel.z()*dt;
	*/


	//double turn_rate = 0.8/param_rate_pose_;
	//Eigen::Quaterniond q_c = Eigen::Quaterniond(Eigen::AngleAxisd(pt_goal_.yaw, Eigen::Vector3d::UnitZ()));
	//Eigen::Quaterniond q_y = Eigen::Quaterniond(odom_current_.pose.pose.orientation.w,
	//											odom_current_.pose.pose.orientation.x,
	//											odom_current_.pose.pose.orientation.y,
	//											odom_current_.pose.pose.orientation.z).normalized().slerp(turn_rate, q_c);

	Eigen::Matrix3d R(q_);
	Eigen::Matrix3d R_sp(q_sp);

    Eigen::Vector3d xb = R.block<3,1>(0,0);
    Eigen::Vector3d xb_sp = R_sp.block<3,1>(0,0);
	double yaw = atan2(xb(1), xb(0));
	double yaw_sp = atan2(xb_sp(1), xb_sp(0));

	//XXX: Handle RP-Yd
	double ye = param_ye_kp_*yaw_error_shortest_path(yaw_sp,yaw);
	if(attitude_goal_.type_mask & attitude_goal_.IGNORE_YAW_RATE) {
		yaw += ye*dt;
	} else {
		yaw += attitude_goal_.body_rate.z*dt;
	}

	Eigen::Vector3d yaw_v( -sin( yaw ), cos( yaw ), 0.0 );
    Eigen::Vector3d zb_sp = R_sp.block<3,1>(0,2);

	R.block<3,1>(0,2) = zb_sp;
	R.block<3,1>(0,0) = yaw_v.cross( zb_sp ).normalized();
	R.block<3,1>(0,1) = R.block<3,1>(0,2).cross( R.block<3,1>(0,0) ).normalized();

	q_ = Eigen::Quaterniond(R).normalized();

	//Calculate acceleration vector
	//Using quadcopter model, so body acceleration is: ba=4F/m
	double z_accel = 0.0;

	z_accel = ( thrust * param_num_motors_ * param_thrust_single_ ) / param_mass_;

	Eigen::Vector3d ba(0.0, 0.0, z_accel );
	Eigen::Vector3d a = R*ba + Eigen::Vector3d(0.0, 0.0, -9.80665);	//world acceleration vector + gravity

	//Propogate linear dynamics
	//Simulate "ground" by putting in a limit / clip
	v_ += a*dt;
	p_ += v_*dt;

	if( p_.z() <= 0.0) {
		p_.z() = 0.0;
		v_.z() = 0.0;
	}

	//==-- Fill out message details
	//Calculate body-frame velocities for odom
	Eigen::Vector3d bv = R.inverse()*v_;

	//Odometery
	nav_msgs::Odometry msg_out_odom;
	msg_out_odom.header.stamp = e.current_real;
	msg_out_odom.header.frame_id = param_frame_id_;
	msg_out_odom.child_frame_id = param_model_id_;

	msg_out_odom.pose.pose.position.x = p_.x();
	msg_out_odom.pose.pose.position.y = p_.y();
	msg_out_odom.pose.pose.position.z = p_.z();
	msg_out_odom.pose.pose.orientation.x = q_.x();
	msg_out_odom.pose.pose.orientation.y = q_.y();
	msg_out_odom.pose.pose.orientation.z = q_.z();
	msg_out_odom.pose.pose.orientation.w = q_.w();
	msg_out_odom.twist.twist.linear.x = bv.x();
	msg_out_odom.twist.twist.linear.y = bv.y();
	msg_out_odom.twist.twist.linear.z = bv.z();
	msg_out_odom.twist.twist.angular.x = 0.0;	//Not worth simulating this with the current setup
	msg_out_odom.twist.twist.angular.y = 0.0;
	msg_out_odom.twist.twist.angular.z = 0.0;

	//Pose
	geometry_msgs::PoseStamped msg_out_pose;
	msg_out_pose.header = msg_out_odom.header;
	msg_out_pose.pose = msg_out_odom.pose.pose;

	//Transform
	geometry_msgs::TransformStamped msg_out_tf;
	msg_out_tf.header = msg_out_odom.header;
	msg_out_tf.child_frame_id = param_model_id_;
	msg_out_tf.transform.translation.x = msg_out_odom.pose.pose.position.x;
	msg_out_tf.transform.translation.y = msg_out_odom.pose.pose.position.y;
	msg_out_tf.transform.translation.z = msg_out_odom.pose.pose.position.z;
	msg_out_tf.transform.rotation = msg_out_odom.pose.pose.orientation;

	//Publish
	pub_pose_.publish(msg_out_pose);
	pub_odom_.publish(msg_out_odom);
	tfbr_.sendTransform(msg_out_tf);
}

void UAVUSREmulator::callback_state(const ros::TimerEvent& e) {
	mavros_msgs::State msg_out_state;
	sensor_msgs::BatteryState msg_out_battery;

	msg_out_state.header.stamp = e.current_real;
	msg_out_state.header.frame_id = param_frame_id_;
	msg_out_state.connected = true;
	msg_out_state.armed = param_system_armed_;
	msg_out_state.guided = true;
	msg_out_state.mode = param_system_mode_;
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
/*
void UAVUSREmulator::callback_goal(const mavros_msgs::PositionTarget::ConstPtr& msg_in) {
	pt_goal_ =*msg_in;
}
*/
void UAVUSREmulator::callback_attitude(const mavros_msgs::AttitudeTarget::ConstPtr& msg_in) {
	attitude_goal_ =*msg_in;
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
	/*
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
	*/
	//If the simulated UAV is above the ground
	if(p_.z() > 0.0) {
		//Fall time calculation
		// z = vz*t - 0.5*g*t^2
		double a = 0.5*GRAV;
		double b = v_.z();
		double c = -p_.z();
		double t = (-b + std::sqrt(b*b - 4*a*c)) / (2*a);

		//ROS_INFO("Drop: %0.4f, %0.4f, %0.4f", a, b, c);
		//ROS_INFO("Hit Time: %0.4f", t);

		hit.position.x = p_.x() + (v_.x() * t);
		hit.position.y = p_.y() + (v_.y() * t);
		hit.position.z = 0.0;
	} else {
		hit.position.x = p_.x();
		hit.position.y = p_.y();
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

nav_msgs::OccupancyGrid UAVUSREmulator::generateGridData( bool gen_random ) {
    nav_msgs::OccupancyGrid msg_out;
    ros::Time stamp = ros::Time::now();

	msg_out.header.frame_id = param_frame_id_;
	msg_out.header.stamp = stamp;
	msg_out.info.map_load_time = stamp;
	msg_out.info.resolution = 0.1;
	msg_out.info.width = 50;
	msg_out.info.height = 50;
	msg_out.info.origin.position.x = -2.5;
	msg_out.info.origin.position.y = -2.5;
	msg_out.info.origin.orientation.w = 1.0;

	int g_obs_x = 20;
	int g_obs_y = 30;
	int g_obs_s = 4;

	std::srand( std::time(0) );

	for(int gv_x = 0; gv_x < msg_out.info.width; gv_x++) {
		for(int gv_y = 0; gv_y < msg_out.info.height; gv_y++) {
			//Generate boundary and obstacle
			int8_t g_val = 0;

			if( (gv_x == 0) ||
				(gv_y == 0) ||
				(gv_x == msg_out.info.width - 1) ||
				(gv_y == msg_out.info.height - 1) ) {
				g_val = 100;
			}

            if (gen_random) {
                if(!( std::rand() % 8 )) {
                    g_val = 100;
                }
            } else {
                if( ( ( gv_x >= g_obs_x - g_obs_s) &&
                      ( gv_x <= g_obs_x + g_obs_s) ) &&
                    ( ( gv_y >= g_obs_y - g_obs_s) &&
                      ( gv_y <= g_obs_y + g_obs_s) ) ) {
                    g_val = 100;
                }
            }

			msg_out.data.push_back(g_val);
        }
    }

    return msg_out;
}

double UAVUSREmulator::clamp(double x, double min, double max) {
    return (x < min) ? min : ( (x > max) ? max : x );
}

};
