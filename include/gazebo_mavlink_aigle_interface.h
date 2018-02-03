#ifndef GAZEBO_MAVLINK_AIGLE_INTERFACE_H
#define GAZEBO_MAVLINK_AIGLE_INTERFACE_H

#include <iostream>
#include <stdio.h>
#include <cstdlib>
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

#include <mavlink/v2.0/common/mavlink.h>

#include "aigle.h"

static const uint32_t kDefaultMavlinkAigleUdpPort = 14570;

namespace gazebo {

class GazeboMavlinkAigleInterface : public ModelPlugin {
public:
	GazeboMavlinkAigleInterface() : ModelPlugin()
	{}

	~GazeboMavlinkAigleInterface();

protected:
	void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
	void OnUpdate(const common::UpdateInfo&  /*_info*/);

private:
	//Pointer to the physic model and world
	physics::ModelPtr model_;
	physics::WorldPtr world_;

	//Pointer to the update event connection
	event::ConnectionPtr updateConnection_;

	//
};

}

#endif // GAZEBO_MAVLINK_AIGLE_INTERFACE_H