#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Image.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace aruco;

image_transport::Publisher debug_pub;
ros::Publisher local_pos_pub;
geometry_msgs::PoseStamped pose;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  debug_pub.publish(msg);
  try
  {
    MarkerDetector MDetector;
    vector<Marker> Markers;
        //read the input image
    cv::Mat InImage=cv_bridge::toCvShare(msg, "bgr8")->image;
    //InImage=cv::imread(argv[1]);
     //Ok, let's detect
    MDetector.detect(InImage,Markers);
        //for each marker, draw info and its boundaries in the image
    for (unsigned int i=0;i<Markers.size();i++) {
        //cout<<Markers[i]<<endl<<endl;
        Markers[i].draw(InImage,Scalar(0,0,255),2);
    }
    if(Markers.size()>0)
    {
        double x = Markers[0][0].x;
        double y = Markers[0][0].y;
        if(x>320+1) pose.pose.position.x+=0.005;
        else if(x<320-1)pose.pose.position.x-=0.005;
        if(y>240+1) pose.pose.position.y-=0.005;
        else if(y<240-1)pose.pose.position.y+=0.005;
        pose.pose.position.z-=0.01;
        cout<<Markers[0][0]<<endl;
        local_pos_pub.publish(pose);
    }

    cv::imshow("aruco",InImage);
    cv::waitKey(30);//wait for key to be pressed
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

/*if (markers->markers.size()>0)
    {

        Matrix<float, 3, 3> Tbn;
        Tbn.setZero();

        Eigen::Quaternionf q(lastImudataReceived.orientation.w, lastImudataReceived.orientation.x, lastImudataReceived.orientation.y, lastImudataReceived.orientation.z);
        Tbn = q.toRotationMatrix();

        Eigen::Vector3f marker_pos_body;
        //convert from ar_track coordinates to body coordinates
        marker_pos_body(0) = -markers->markers[0].pose.pose.position.y;
        marker_pos_body(1) = -markers->markers[0].pose.pose.position.x;
        marker_pos_body(2) = -markers->markers[0].pose.pose.position.z;

        Eigen::Vector3f marker_pos_NED;
        marker_pos_NED.setZero();

        Eigen::Vector3f pos;
        pos.setZero();
        pos = marker_pos_NED - Tbn*marker_pos_body;


        //publish marker pose
        geometry_msgs::PoseStamped pose_msg;
        if(convertToNED)
        {
            pose_msg.pose.position.x = pos(0);
            pose_msg.pose.position.y = pos(1);
            pose_msg.pose.position.z = pos(2);
        }else
        {
            pose_msg.pose.position.x = marker_pos_body(0);
            pose_msg.pose.position.y = marker_pos_body(1);
            pose_msg.pose.position.z = marker_pos_body(2);
        }
        pose_msg.header.stamp = ros::Time::now();
        pose_pub.publish(pose_msg);
        ROS_INFO("ARTag Id=%d x=%.2f y=%.2f z=%.2f",
                 markers->markers[0].id,
                pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
    }
    */


int main(int argc,char **argv)
{
    ros::init(argc, argv, "aruco_listener");
    ros::NodeHandle nh;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 4;
    cv::namedWindow("aruco");
    cv::startWindowThread();
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/autonomy_landing/marker_pose", 10);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/autonomy_landing/camera1/image_raw", 1, imageCallback);

    debug_pub = it.advertise("/image", 1);
    ros::spin();
    cv::destroyWindow("aruco");
}