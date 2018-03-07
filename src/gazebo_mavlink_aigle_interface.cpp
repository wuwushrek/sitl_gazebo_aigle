
#include <gazebo_mavlink_aigle_interface.h>

namespace gazebo{

GZ_REGISTER_MODEL_PLUGIN(GazeboMavlinkAigleInterface);

int receive_fd();

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

	const char *env_alt = std::getenv("PX4_HOME_ALT");
	if (env_alt) {
		gzmsg << "Home altitude is set to " << env_alt << ".\n";
		alt_home = std::stod(env_alt);
	}

	// Init the node handle for subscriber and publisher 
	node_handle_ = transport::NodePtr(new transport::Node());
	node_handle_->Init(namespace_);

	// get GPS topic name frm sdf file
	getSdfParam<std::string>(_sdf, "gpsSubTopic", gps_sub_topic_, gps_sub_topic_);

	// Get IMU topic name from sdf file
  getSdfParam<std::string>(_sdf, "imuSubTopic", imu_sub_topic_, imu_sub_topic_);
  groundtruth_sub_topic_ = "/groundtruth";

  if (_sdf->HasElement("mavlink_aigle_udp_port")) {
    aigle_port = _sdf->GetElement("mavlink_aigle_udp_port")->Get<int>();
  }

  // Initialize aigle destination address
  memset( (char * )&srcaddr_ , 0 , sizeof(srcaddr_));
  srcaddr_.sin_family = AF_INET;
  srcaddr_.sin_addr.s_addr =  htonl(INADDR_ANY);
  srcaddr_.sin_port = htons(aigle_port);

  // Get file descriptor for aigle port from the vehicle plugin
  _fd_aigle = receive_fd(); 

	// // Magnetic field data for Zurich from WMM2015 (10^5xnanoTesla (N, E D) n-frame )
	// // mag_n_ = {0.21523, 0.00771, -0.42741};
	// // We set the world Y component to zero because we apply
	// // the declination based on the global position,
	// // and so we need to start without any offsets.
	// // The real value for Zurich would be 0.00771
	// // frame d is the magnetic north frame
	// mag_d_.x = 0.21523;
	// mag_d_.y = 0;
	// mag_d_.z = -0.42741;

  mag_d_.x = 1.0;
  mag_d_.y = 0;
  mag_d_.z = 0.0;

	//	Used for altitude estimation 
	gravity_W_ = world_->GetPhysicsEngine()->GetGravity();

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	updateConnection_ = event::Events::ConnectWorldUpdateBegin( boost::bind(&GazeboMavlinkAigleInterface::OnUpdate, this, _1));

	//Subscribe to GPS data
	gps_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + gps_sub_topic_, &GazeboMavlinkAigleInterface::GpsCallback, this);
	imu_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + imu_sub_topic_, &GazeboMavlinkAigleInterface::ImuCallback, this);
	groundtruth_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + groundtruth_sub_topic_, &GazeboMavlinkAigleInterface::GroundtruthCallback, this);
}

void GazeboMavlinkAigleInterface::OnUpdate(const common::UpdateInfo&  /*_info*/) {

	// Execute AIGLE strategy by reading motors value if availabled
	// And writing motors output depending of the current state of the robot
	// Cf aigle.h & aigle.cpp documentation for more details
	// execute_strategy();
}

void GazeboMavlinkAigleInterface::GpsCallback(GpsPtr& gps_msg){
	mavlink_hil_gps_t m_gps_position;
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

  // send HIL_GPS Mavlink msg
  mavlink_message_t msg;
  mavlink_msg_hil_gps_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &m_gps_position);
  send_mavlink_message(&msg);

	//Update aigle_io GPS position
	// aigle_io.updateGPS(&m_gps_position);
}

void GazeboMavlinkAigleInterface::GroundtruthCallback(GtPtr& groundtruth_msg){
  // update groundtruth lat_rad, lon_rad and altitude
  groundtruth_lat_rad = groundtruth_msg->latitude_rad();
  groundtruth_lon_rad = groundtruth_msg->longitude_rad();
  groundtruth_altitude = groundtruth_msg->altitude();
  // the rest of the data is obtained directly on this interface and sent to
  // the FCU
}

void GazeboMavlinkAigleInterface::ImuCallback(ImuPtr& imu_message) {
  // frames
  // g - gazebo (ENU), east, north, up
  // r - rotors imu frame (FLU), forward, left, up
  // n - px4  (NED)
  math::Quaternion q_gr = math::Quaternion(
      imu_message->orientation().w(),
      imu_message->orientation().x(),
      imu_message->orientation().y(),
      imu_message->orientation().z());

  math::Vector3 pos_g = model_->GetWorldPose().pos;

  //gzerr << "got imu: " << C_W_I << "\n";
  //gzerr << "got pose: " << T_W_I.rot << "\n";
  // float declination = get_mag_declination(groundtruth_lat_rad, groundtruth_lon_rad);

  // math::Quaternion q_dn(0.0, 0.0, declination);
  // math::Vector3 mag_n = q_dn.RotateVector(q_ng. mag_d_);

  math::Vector3 vel_r = model_->GetRelativeLinearVel();
  math::Vector3 vel_g = model_->GetWorldLinearVel();
  math::Vector3 omega_nb_g = model_->GetRelativeAngularVel();

  math::Vector3 mag_noise_b(
      0.01 * randn_(rand_),
      0.01 * randn_(rand_),
      0.01 * randn_(rand_));

  math::Vector3 accel_r = math::Vector3(
        imu_message->linear_acceleration().x(),
        imu_message->linear_acceleration().y(),
        imu_message->linear_acceleration().z());
  math::Vector3 gyro_r = math::Vector3(
        imu_message->angular_velocity().x(),
        imu_message->angular_velocity().y(),
        imu_message->angular_velocity().z());
  math::Vector3 mag_r = q_gr.RotateVectorReverse(mag_d_) + mag_noise_b;
  // gzerr << mag_b << std::endl;

  mavlink_hil_sensor_t sensor_msg;
  sensor_msg.time_usec = world_->GetSimTime().Double() * 1e6;
  sensor_msg.xacc = accel_r.x;
  sensor_msg.yacc = accel_r.y;
  sensor_msg.zacc = accel_r.z;
  sensor_msg.xgyro = gyro_r.x;
  sensor_msg.ygyro = gyro_r.y;
  sensor_msg.zgyro = gyro_r.z;
  sensor_msg.xmag = mag_r.x;
  sensor_msg.ymag = mag_r.y;
  sensor_msg.zmag = mag_r.z;

  // calculate abs_pressure using an ISA model for the tropsphere (valid up to 11km above MSL)
  const float lapse_rate = 0.0065f;  // reduction in temperature with altitude (Kelvin/m)
  const float temperature_msl = 288.0f;  // temperature at MSL (Kelvin)
  float alt_msl = (float)alt_home - pos_g.z;
  float temperature_local = temperature_msl - lapse_rate * alt_msl;
  float pressure_ratio = powf((temperature_msl / temperature_local), 5.256f);
  const float pressure_msl = 101325.0f;  // pressure at MSL
  sensor_msg.abs_pressure = pressure_msl / pressure_ratio;

  // generate Gaussian noise sequence using polar form of Box-Muller transformation
  // http://www.design.caltech.edu/erik/Misc/Gaussian.html
  double x1, x2, w, y1, y2;
  do {
    x1 = 2.0 * (rand() * (1.0 / (double)RAND_MAX)) - 1.0;
    x2 = 2.0 * (rand() * (1.0 / (double)RAND_MAX)) - 1.0;
    w = x1 * x1 + x2 * x2;
  } while ( w >= 1.0 );
  w = sqrt( (-2.0 * log( w ) ) / w );
  y1 = x1 * w;
  y2 = x2 * w;

  // Apply 1 Pa RMS noise
  float abs_pressure_noise = 1.0f * (float)w;
  sensor_msg.abs_pressure += abs_pressure_noise;

  // convert to hPa
  sensor_msg.abs_pressure *= 0.01f;

  // calculate density using an ISA model for the tropsphere (valid up to 11km above MSL)
  const float density_ratio = powf((temperature_msl / temperature_local), 4.256f);
  float rho = 1.225f / density_ratio;

  // calculate pressure altitude including effect of pressure noise
  sensor_msg.pressure_alt = alt_msl - abs_pressure_noise / (gravity_W_.GetLength() * rho);

  // calculate differential pressure in hPa
  sensor_msg.diff_pressure = 0.005f * rho * vel_r.x * vel_r.x;

  // calculate temperature in Celsius
  sensor_msg.temperature = temperature_local - 273.0f;

  sensor_msg.fields_updated = 4095;

  mavlink_message_t msg;
  mavlink_msg_hil_sensor_encode_chan(1, 200, MAVLINK_COMM_0, &msg, &sensor_msg);
  send_mavlink_message(&msg);
  // aigle_io.updateIMU(&sensor_msg);

}

void GazeboMavlinkAigleInterface::send_mavlink_message(const mavlink_message_t *message)
{
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  int packetlen = mavlink_msg_to_send_buffer(buffer, message);
  ssize_t len = sendto(_fd_aigle, buffer, packetlen, 0, (struct sockaddr *) &srcaddr_, sizeof(srcaddr_));
  // ssize_t len = sendto(_fd_aigle, buffer, packetlen, 0);

  if (len <= 0) {
    printf("Failed sending mavlink message\n");
  }
}

/*
* Another Routine to receive the file descriptor from another process
* Here the file descriptor will point at the mavlink port to receive/send motors/imu,gps data
*/
int receive_fd(){

  // Create a UNIX socket and the addresses associated with the socket
  int fd_unix;
  struct sockaddr_un send_addr;

  // Try to initalize the socket as TCP socket
  if((fd_unix = socket(AF_UNIX, SOCK_STREAM , 0)) < 0){
    gzerr << "[AIGLE] Failed to create Local UNIX socket !" << std::endl;
    return -1;
  }

  // Initialize the current socket address 
  memset(&send_addr , 0 , sizeof(struct sockaddr_un));
  send_addr.sun_family =  AF_UNIX;
  strcpy(send_addr.sun_path , SOCKET_NAME_AIGLE_PLUGIN);

  // Try to connect to the local server
  while ( connect(fd_unix , (struct sockaddr *) &send_addr , ( strlen(send_addr.sun_path) + sizeof(send_addr.sun_family))) < 0);
  gzmsg << " Connected to local socket unix \n";
  // Define structure for sending file descriptor amongst different process
  struct msghdr msg = {0};
  char useless[4];

  //Structure buffer for sending message with ancillary datas
  struct iovec io[1];
  io[0].iov_base = (void *) useless;
  io[0].iov_len = sizeof(useless);

  // Fill the message structure with the message we want to send
  msg.msg_iov = io;
  msg.msg_iovlen = 1;

  // Initialize the anicllary data structure
  char buf[256];
  msg.msg_control = buf;
  msg.msg_controllen = sizeof(buf);

  // Try to receive the message with the file descriptor information
  if (recvmsg(fd_unix,&msg,0) < 0) {
    gzmsg << "[AIGLE] Trying to get fd from robot plugin !!" << std::endl;
  }

  // Extract the ancillary data we wanted to obtain
  struct cmsghdr *cmsg = CMSG_FIRSTHDR(&msg);
  int result;
  memmove(&result, CMSG_DATA(cmsg), sizeof(result));

  gzmsg << "[AIGLE] Aigle port socket fd = " << result <<  std::endl;
  // Close the socket since not use anymore
  close(fd_unix);
  return result;
}

}
