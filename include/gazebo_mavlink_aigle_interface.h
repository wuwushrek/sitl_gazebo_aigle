#ifndef GAZEBO_MAVLINK_AIGLE_INTERFACE_H
#define GAZEBO_MAVLINK_AIGLE_INTERFACE_H

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <random>
#include <math.h>
#include <string>
#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <sdf/sdf.hh>
#include <common.h>
#include <SITLGps.pb.h>
#include <SensorImu.pb.h>
#include <Groundtruth.pb.h>

#include <mavlink/v2.0/common/mavlink.h>

#include <geo_mag_declination.h>

#include "aigle.h"

static const uint32_t kDefaultMavlinkAigleUdpPort = 14570;

namespace gazebo {


typedef const boost::shared_ptr<const gps_msgs::msgs::SITLGps> GpsPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Imu> ImuPtr;
typedef const boost::shared_ptr<const gps_msgs::msgs::Groundtruth> GtPtr;

class GazeboMavlinkAigleInterface : public ModelPlugin {
public:
	GazeboMavlinkAigleInterface() : ModelPlugin(),
	namespace_("")
	{}

	~GazeboMavlinkAigleInterface();

protected:
	void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
	void OnUpdate(const common::UpdateInfo&  /*_info*/);

private:
	//Pointer to the physic model and world
	physics::ModelPtr model_;
	physics::WorldPtr world_;

	//Node handle for topic subscribe and publish
	transport::NodePtr node_handle_;
	std::string namespace_;

	//Pointer to the update event connection
	event::ConnectionPtr updateConnection_;

	// GPS subscriber and gps subscriber topic
  	std::string gps_sub_topic_;
  	std::string groundtruth_sub_topic_;
  	transport::SubscriberPtr gps_sub_;
  	transport::SubscriberPtr groundtruth_sub_;

  	// IMU Subscriber and imu subscriber topic
  	std::string imu_sub_topic_;
  	transport::SubscriberPtr imu_sub_;

	void GpsCallback(GpsPtr& gps_msg);
	void ImuCallback(ImuPtr& imu_msg);
	void GroundtruthCallback(GtPtr& groundtruth_msg);

	// For baro estimation
	double alt_home = 488.0;   // meters

	double groundtruth_lat_rad;
	double groundtruth_lon_rad;
	double groundtruth_altitude;

	math::Vector3 gravity_W_;
	math::Vector3 mag_d_;

	std::default_random_engine rand_;
	std::normal_distribution<float> randn_;
};

}

#endif // GAZEBO_MAVLINK_AIGLE_INTERFACE_H