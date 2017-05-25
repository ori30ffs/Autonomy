#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Image.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv; 
image_transport::Publisher debug_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  debug_pub.publish(msg);
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/autonomy_landing/camera1/image_raw", 1, imageCallback);

  debug_pub = it.advertise("/image", 1);
  ros::spin();
  cv::destroyWindow("view");
}