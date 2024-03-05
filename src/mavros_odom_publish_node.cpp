#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class MavrosOdomPublisher
{
public:
  MavrosOdomPublisher(const std::string &vehicle_type, const std::string &vehicle_id)
      : _vehicle_type(vehicle_type), _vehicle_id(vehicle_id)
  {
    _odom_pub = _n.advertise<nav_msgs::Odometry>("odom", 5);

    _odom_sub = _n.subscribe("mavros/local_position/odom", 10, &MavrosOdomPublisher::odomCallback, this);
    
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    odom_msg_ = *msg;
    odom_msg_.header.frame_id = "odom"; 
    _odom_pub.publish(odom_msg_);

    odom_tf_.header = odom_msg_.header; 
    odom_tf_.child_frame_id = "base_link";
    odom_tf_.transform.translation.x = odom_msg_.pose.pose.position.x;
    odom_tf_.transform.translation.y = odom_msg_.pose.pose.position.y;
    odom_tf_.transform.translation.z = odom_msg_.pose.pose.position.z;
    odom_tf_.transform.rotation = odom_msg_.pose.pose.orientation;

    tf_broadcaster_.sendTransform(odom_tf_);
  }

private:
  ros::NodeHandle _n;
  ros::Publisher _odom_pub;
  ros::Subscriber _odom_sub;
  std::string _vehicle_type;
  std::string _vehicle_id;
  nav_msgs::Odometry odom_msg_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped odom_tf_;
}; 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_publisher");
  MavrosOdomPublisher odomObject(argv[1], argv[2]);
  ros::spin();
  return 0;
}
