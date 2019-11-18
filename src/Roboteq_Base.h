
#ifndef ROBOTQE_BASE
#define ROBOTEQ_BASE

#include <ros/ros.h>
#include <ros/console.h>
#include <serial/serial.h>
#include <signal.h>
#include <string>
#include <sstream>
#include <algorithm>

#define DELTAT(_nowtime,_thentime) ((_thentime>_nowtime)?((0xffffffff-_thentime)+_nowtime):(_nowtime-_thentime))

//
// cmd_vel subscriber
//

// Define following to enable cmdvel debug output
#define _CMDVEL_DEBUG

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


//
// odom publisher
//

// Define following to enable odom debug output
#define _ODOM_DEBUG

// Define following to publish additional sensor information
#define _ODOM_SENSORS

// Define following to enable service for returning covariance
//#define _ODOM_COVAR_SERVER

#define NORMALIZE(_z) atan2(sin(_z), cos(_z))

#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#ifdef _ODOM_SENSORS
#include <std_msgs/Float32.h>
#include <roboteq_diff_msgs/Duplex.h>
#endif
#ifdef _ODOM_COVAR_SERVER
#include "roboteq_diff_msgs/OdometryCovariances.h"
#include "rogoteq_diff_msgs/RequestOdometryCovariances.h"
#endif

class Roboteq {} 

friend class Protocol;

public:

  //
  // cmd_vel subscriber
  //
  void cmdvel_callback( const geometry_msgs::Twist& twist_msg); // set_motor_rpm
  void cmdvel_setup(serial::Serial& controller); // Set_operating_mode, set_motor_amps, set_max_rpm, set_max_accel, set_max_decel, set_pid_controls, set_encoder_mode, set_encoder_counts
  void cmdvel_loop();
  void cmdvel_run(); // start motors

  //
  // odom publisher
  //
  void odom_setup();
  void odom_stream();
  void odom_loop(); // read -> encoder counts (CR=), voltage (V), current (BA=), 
  //void odom_hs_run();
  void odom_ms_run(); // publish -> current
  void odom_ls_run(); // publish -> voltage, energy, temp
  void odom_publish(); // calculate velocities
#ifdef _ODOM_COVAR_SERVER // delete
  void odom_covar_callback(const roboteq_diff_msgs::RequestOdometryCovariancesRequest& req, roboteq_diff_msgs::RequestOdometryCovariancesResponse& res);
#endif

  int run();

protected:
  
 // sensors
#ifdef _ODOM_SENSORS
  float voltage;
  float current_right;
  float current_left;
  float energy;
  float temperature;
  uint32_t current_last_time;
#endif

#ifdef _ODOM_SENSORS
  std_msgs::Float32 voltage_msg;
  ros::Publisher voltage_pub;
  roboteq_diff_msgs::Duplex current_msg;
  ros::Publisher current_pub;
  std_msgs::Float32 energy_msg;
  ros::Publisher energy_pub;
  std_msgs::Float32 temperature_msg;
  ros::Publisher temperature_pub;
#endif

  // settings
  bool pub_odom_tf;
  std::string odom_frame;
  std::string base_frame;
  std::string cmdvel_topic;
  std::string odom_topic;
  std::string port_back;
  std::string port_right;
  std::string port_left;
  int baud;
  bool open_loop;
  double wheel_circumference;
  double track_width;
  int encoder_ppr;
  int encoder_cpr;
  int kp;
  int ki;
  int kd;	
  float gear_ratio;
  
  // motor commands
   void setEstop() { command << "EX" << send; }
   void resetEstop() { command << "MG" << send; }
   void resetDIOx(int i) { command << "D0" << i << send; }
   void setDIOx(int i) { command << "D1" << i << send; }
   void startScript() { command << "R" << send; }
   void stopScript() { command << "R" << 0 << send; }
   void setUserVariable(int var, int val) { command << "VAR" << var << val << send; }

  ros::NodeHandle nh;

  serial::Serial controller_back;
  serial::Serial controller_right;
  serial::Serial controller_left;

  uint32_t starttime;
  uint32_t hstimer;
  uint32_t mstimer;
  uint32_t lstimer;

  //
  // cmd_vel subscriber
  //
  ros::Subscriber cmdvel_sub;

  //
  // odom publisher
  //
  geometry_msgs::TransformStamped tf_msg;
  tf::TransformBroadcaster odom_broadcaster;
  nav_msgs::Odometry odom_msg;
  ros::Publisher odom_pub;

  // buffer for reading encoder counts
  int odom_idx;
  char odom_buf[24];

  // toss out initial encoder readings
  char odom_encoder_toss;

  int32_t odom_encoder_left;
  int32_t odom_encoder_right;

  float odom_x;
  float odom_y;
  float odom_yaw;
  float odom_last_x;
  float odom_last_y;
  float odom_last_yaw;

  uint32_t odom_last_time;

};