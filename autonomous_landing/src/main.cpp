#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

int main(int argc, char **argv)
{
	/*ros::init(argc, argv, "ros_gazebo_connection_node");
	ros::NodeHandle n;
	ROS_INFO("SIMULATION GUIT START NOW\n");
	ros::spin();*/
	ROS_INFO("SIMULATION GUIT START NOW\n");
	std::unique_ptr<ros::NodeHandle> rosNode;
	ros::Subscriber rosSub;
	ros::CallbackQueue rosQueue;
	std::thread rosQueueThread;

	if (!ros::isInitialized())
	{
		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, "gazebo_client",	ros::init_options::NoSigintHandler);
	}

	// Create our ROS node. This acts in a similar manner to
	// the Gazebo node
	rosNode.reset(new ros::NodeHandle("gazebo_client"));
	ros::spin();
	// Create a named topic, and subscribe to it.
	//ros::SubscribeOptions so =
	//ros::SubscribeOptions::create<std_msgs::Float32>("/" + model->GetName() + "/vel_cmd",	1,	boost::bind(&VelodynePlugin::OnRosMsg, this, _1), ros::VoidPtr(), &this->rosQueue);
	//rosSub = rosNode->subscribe(so);

	// Spin up the queue helper thread.
	//this->rosQueueThread =	std::thread(std::bind(&VelodynePlugin::QueueThread, this));

	return 0;
}
