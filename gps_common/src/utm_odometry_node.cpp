/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <rclcpp/rclcpp.hpp>
//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <gps_common/conversions.h>
#include <nav_msgs/msg/odometry.hpp>

using namespace gps_common;

static rclcpp::Publisher<nav_msgs::msg::Odometry> odom_pub;
rclcpp::Node::SharedPtr node = nullptr;
std::string frame_id, child_frame_id;
double rot_cov;

void callback(const sensor_msgs::msg::NavSatFix::SharedPtr fix) {
  if (fix->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    RCLCPP_DEBUG(node->get_logger(),"No fix.");
    return;
  }

  if (fix->header.stamp == node->now()) {
    return;
  }

  double northing, easting;
  std::string zone;

  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

  if (odom_pub) {
    nav_msgs::msg::Odometry::SharedPtr odom;
    odom->header.stamp = fix->header.stamp;

    if (frame_id.empty())
      odom->header.frame_id = fix->header.frame_id;
    else
      odom->header.frame_id = frame_id;

    odom->child_frame_id = child_frame_id;

    odom->pose.pose.position.x = easting;
    odom->pose.pose.position.y = northing;
    odom->pose.pose.position.z = fix->altitude;

    odom->pose.pose.orientation.x = 0;
    odom->pose.pose.orientation.y = 0;
    odom->pose.pose.orientation.z = 0;
    odom->pose.pose.orientation.w = 1;

    // Use ENU covariance to build XYZRPY covariance
    boost::array<double, 36> covariance = {{
      fix->position_covariance[0],
      fix->position_covariance[1],
      fix->position_covariance[2],
      0, 0, 0,
      fix->position_covariance[3],
      fix->position_covariance[4],
      fix->position_covariance[5],
      0, 0, 0,
      fix->position_covariance[6],
      fix->position_covariance[7],
      fix->position_covariance[8],
      0, 0, 0,
      0, 0, 0, rot_cov, 0, 0,
      0, 0, 0, 0, rot_cov, 0,
      0, 0, 0, 0, 0, rot_cov
    }};

    odom->pose.covariance = covariance;

    odom_pub->publish(*odom);
  }
}

int main (int argc, char **argv) {
  rclcpp::init(argc, argv);
  // set the node
  node = rclcpp::Node::make_shared("utm_odometry_node");
  node->declare_parameter("frame_id","");
  node->declare_parameter("child_frame_id","");
  node->declare_parameter("rot_covariance",99999.0);

  frame_id = (node->get_parameter("frame_id")).as_string();
  child_frame_id = (node->get_parameter("child_frame_id")).as_string();
  rot_cov = (node->get_parameter("rot_covariance").as_double());

  odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));

  auto fix_sub = node->create_subscription<sensor_msgs::msg::NavSatFix>("fix",
    rclcpp::QoS(10),
    callback
  );

  rclcpp::spin(node);
  rclcpp::shutdown();
  // needed? see https://github.com/ros2/examples/blob/master/rclcpp/minimal_subscriber/not_composable.cpp
  subscription = nullptr;
  node = nullptr;
  odom_pub = nullptr;
  return 0;
}
