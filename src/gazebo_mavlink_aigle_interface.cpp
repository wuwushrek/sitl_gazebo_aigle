
#include <gazebo_mavlink_aigle_interface.h>

namespace gazebo{

GZ_REGISTER_MODEL_PLUGIN(GazeboMavlinkAigleInterface);

GazeboMavlinkAigleInterface::~GazeboMavlinkAigleInterface() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
}

void GazeboMavlinkAigleInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

	//Store the pointer to the model and the world instance
	model_ =  _model;
	world_ = model_->GetWorld();

	// Get the namespace_ if there is any
	namespace_.clear();
	if (_sdf->HasElement("robotNamespace")) {
		namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
	} else {
		gzerr << "[gazebo_mavlink_aigle_interface] Please specify a robotNamespace.\n";
	}

	// Init the node handle for subscriber and publisher 
	node_handle_ = transport::NodePtr(new transport::Node());
	node_handle_->Init(namespace_);

	// get GPS topic name frm sdf file
	getSdfParam<std::string>(_sdf, "gpsSubTopic", gps_sub_topic_, gps_sub_topic_);

	// Initialize AIGLE depending on the port value received in the sdf file
	if (_sdf->HasElement("mavlink_aigle_udp_port")){
		init_aigle(_sdf->GetElement("mavlink_aigle_udp_port")->Get<int>());
	} else {
		init_aigle(kDefaultMavlinkAigleUdpPort);
	}

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	updateConnection_ = event::Events::ConnectWorldUpdateBegin( boost::bind(&GazeboMavlinkAigleInterface::OnUpdate, this, _1));

	//Subscribe to GPS data
	gps_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + gps_sub_topic_, &GazeboMavlinkAigleInterface::GpsCallback, this);

}

void GazeboMavlinkAigleInterface::OnUpdate(const common::UpdateInfo&  /*_info*/) {
	
	// Execute AIGLE strategy by reading motors value if availabled
	// And writing motors output depending of the current state of the robot
	// Cf aigle.h & aigle.cpp documentation for more details
	execute_strategy(); 
}

void GazeboMavlinkAigleInterface::GpsCallback(GpsPtr& gps_msg){
	gps_data m_gps_position;
	m_gps_position.time_usec = gps_msg->time() * 1e6;
	m_gps_position.fix_type = 3;
	m_gps_position.lat = gps_msg->latitude_deg() * 1e7;
	m_gps_position.lon = gps_msg->longitude_deg() * 1e7;
	m_gps_position.alt = gps_msg->altitude() * 1000.0;
	m_gps_position.eph = gps_msg->eph() * 100.0;
	m_gps_position.epv = gps_msg->epv() * 100.0;
	m_gps_position.vel = gps_msg->velocity() * 100.0;
	m_gps_position.vn = gps_msg->velocity_north() * 100.0;
	m_gps_position.ve = gps_msg->velocity_east() * 100.0;
	m_gps_position.vd = -gps_msg->velocity_up() * 100.0;
	// MAVLINK_HIL_GPS_T CoG is [0, 360]. math::Angle::Normalize() is [-pi, pi].
	math::Angle cog(atan2(gps_msg->velocity_east(), gps_msg->velocity_north()));
	cog.Normalize();
	m_gps_position.cog = static_cast<uint16_t>(GetDegrees360(cog) * 100.0);
	m_gps_position.satellites_visible = 10;

	//Update aigle_io GPS position
	aigle_io.updateGPS(&m_gps_position);
}

}