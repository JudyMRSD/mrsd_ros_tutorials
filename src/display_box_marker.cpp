#include <iostream>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>

#include <dynamic_reconfigure/server.h>
#include <mrsd_ros_tutorials/TransformsConfig.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

geometry_msgs::Transform g_tf;

void reconfig_callback(mrsd_ros_tutorials::TransformsConfig &config) {
  std::cout << "Got a callback from reconfigure\n";
  // Set Translation
  g_tf.translation.x = config.x;
  g_tf.translation.y = config.y;
  g_tf.translation.z = config.z;

  // Set Rotation
  Eigen::AngleAxisd roll = Eigen::AngleAxisd(config.roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch = Eigen::AngleAxisd(config.pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw = Eigen::AngleAxisd(config.yaw, Eigen::Vector3d::UnitZ());

  Eigen::Quaterniond QEigen;
  QEigen = roll * pitch * yaw;

  //tf::Quaternion Qtf;
  //tf::quaternionEigenToTF(QEigen, Qtf);
  
  g_tf.rotation.x = QEigen.x();
  g_tf.rotation.y = QEigen.y();
  g_tf.rotation.z = QEigen.z();
  g_tf.rotation.w = QEigen.w();

}


class BoxDisplayer
{
public:

  ros::NodeHandle nh_;
  geometry_msgs::Pose box_pose_;
  geometry_msgs::TransformStamped box_tf_;
  ros::Publisher marker_pub_;
  visualization_msgs::Marker marker_;
  tf::TransformBroadcaster tf_broadcaster_;

  BoxDisplayer()
  {
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("box_marker", 1);
    marker_.header.frame_id = "/box";
    //marker_.header.stamp = ros::Time::now();
    marker_.ns = "shelfbin_marker";
    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_.mesh_resource = "package://mrsd_ros_tutorials/model/bin.STL";

    marker_.action = visualization_msgs::Marker::ADD;

    marker_.scale.x = 1.0;
    marker_.scale.y = 1.0;
    marker_.scale.z = 1.0;

    marker_.color.r = 0.0f;
    marker_.color.g = 1.0f;
    marker_.color.b = 0.0f;
    marker_.color.a = 1.0;

    marker_.lifetime = ros::Duration();

    box_pose_.orientation.w = 1; // so inital pose is a valid pose (magnitude of Quaternion should = 1)
    marker_.pose = box_pose_;
    std::cout << "Class initialized\n";
    box_tf_.transform.rotation.w = 1;
    box_tf_.header.stamp = ros::Time::now();
    box_tf_.header.frame_id = "world";
    box_tf_.child_frame_id = "box";
    tf_broadcaster_.sendTransform(box_tf_);

  }


  void updateBox(const geometry_msgs::Transform input_tf)
  {
    box_tf_.transform = input_tf;
    //std::cout << "updating Box\n";
    marker_.header.stamp = ros::Time::now();
    marker_pub_.publish(marker_);

    box_tf_.header.stamp = ros::Time::now();
    box_tf_.header.frame_id = "world";
    box_tf_.child_frame_id = "box";

    tf_broadcaster_.sendTransform(box_tf_);
  }


}; // Class Definition


int main( int argc, char** argv )
{
  ros::init(argc, argv, "display_box_marker");
  ros::Time::init();
  g_tf.rotation.w = 1;
  ros::Rate rate(10);
  ros::NodeHandle nh;
  BoxDisplayer BD;


  dynamic_reconfigure::Server<mrsd_ros_tutorials::TransformsConfig> server;
  dynamic_reconfigure::Server<mrsd_ros_tutorials::TransformsConfig>::CallbackType cbType;

  cbType = boost::bind(&reconfig_callback, _1);
  server.setCallback(cbType);

  std::cout << "Initialized display_box_marker node\n";


  while (ros::ok() )
  {
    BD.updateBox(g_tf);
    rate.sleep();
  }
  return 0;
}
