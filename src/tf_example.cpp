#include <ros/ros.h>



int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_example");

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;


  while(ros::ok() )
  {
    ros::Duration(.01).sleep();
    ros::spinOnce();
  }
  return 0;
}