
#include <gazebo_mavlink_aigle_interface.h>

namespace gazebo{

GZ_REGISTER_MODEL_PLUGIN(GazeboMavlinkAigleInterface);

GazeboMavlinkAigleInterface::~GazeboMavlinkAigleInterface() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
}

void GazeboMavlinkAigleInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

	//Store the pointer to the model
	model_ =  _model;

	world_ = model_->GetWorld();

	// Initialize AIGLE depending on the port value received in the sdf file
	gzmsg << "Before Aigle initialization !" << std::endl;
	if (_sdf->HasElement("mavlink_aigle_udp_port")){
		init_aigle(_sdf->GetElement("mavlink_aigle_udp_port")->Get<int>());
	} else {
		init_aigle(kDefaultMavlinkAigleUdpPort);
	}
	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	updateConnection_ = event::Events::ConnectWorldUpdateBegin( boost::bind(&GazeboMavlinkAigleInterface::OnUpdate, this, _1));
	
}

void GazeboMavlinkAigleInterface::OnUpdate(const common::UpdateInfo&  /*_info*/) {
	
	// Execute AIGLE strategy by reading motors value if availabled
	// And writing motors output depending of the current state of the robot
	// Cf aigle.h & aigle.cpp documentation for more details
	execute_strategy(); 
}

}