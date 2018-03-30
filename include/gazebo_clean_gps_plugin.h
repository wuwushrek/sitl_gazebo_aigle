#ifndef _GAZEBO_CLEAN_GPS_PLUGIN_HH_
#define _GAZEBO_CLEAN_GPS_PLUGIN_HH_

#include <math.h>
#include <cstdio>
#include <cstdlib>
#include <queue>
#include <random>

#include <sdf/sdf.hh>
#include <common.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/math/gzmath.hh>

#include <SITLGps.pb.h>
#include <Groundtruth.pb.h>

namespace gazebo
{
class GAZEBO_VISIBLE CleanGpsPlugin : public ModelPlugin
{
public:
  CleanGpsPlugin();
  virtual ~CleanGpsPlugin();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo&);

private:
  std::pair<double, double> reproject(math::Vector3& pos);

  std::string namespace_;

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr updateConnection_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr gt_pub_;
  transport::PublisherPtr gps_pub_;

  gps_msgs::msgs::SITLGps gps_msg;
  gps_msgs::msgs::Groundtruth groundtruth_msg;

  common::Time last_gps_time_;
  common::Time last_time_;

  // Set global reference point
  // Zurich Irchel Park: 47.397742, 8.545594, 488m
  // Seattle downtown (15 deg declination): 47.592182, -122.316031, 86m
  // Moscow downtown: 55.753395, 37.625427, 155m

  // The home position can be specified using the environment variables:
  // PX4_HOME_LAT, PX4_HOME_LON, and PX4_HOME_ALT

  // Zurich Irchel Park
  double lat_home = 47.397742 * M_PI / 180.0;  // rad
  double lon_home = 8.545594 * M_PI / 180.0;   // rad
  double alt_home = 488.0;                     // meters
  // Seattle downtown (15 deg declination): 47.592182, -122.316031
  // static const double lat_home = 47.592182 * M_PI / 180;    // rad
  // static const double lon_home = -122.316031 * M_PI / 180;  // rad
  // static const double alt_home = 86.0;                      // meters

  static constexpr const double earth_radius = 6353000.0;      // meters

  math::Vector3 gravity_W_;
  math::Vector3 velocity_prev_W_;
};
}
#endif
