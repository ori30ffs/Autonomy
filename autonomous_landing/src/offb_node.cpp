#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float64.h"
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Publisher pub_att = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude",100);
    ros::Publisher pub_thr = nh.advertise<std_msgs::Float64>("/mavros/setpoint_attitude/att_throttle", 100);
   
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    /**************************************************************************
    **********************************MOVING***********************************           
    **************************************************************************/
    geometry_msgs::PoseStamped cmd_att;
    std_msgs::Float64 cmd_thr;
    int count = 1;
    double v[3]={1.0, 0.0, 0.0};
    double lambda = 0.3;
    double v_norm=sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    double theta=0.0;
    /*************************************************************************/
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 4;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    int move = 0;
    float phi =0;
    int r = 2;

    while(ros::ok())
    {
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else 
        {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    move = 1;
                }
                last_request = ros::Time::now();
            }
        }
        if(move == 1)
        {
            /*cmd_att.header.stamp = ros::Time::now();
            cmd_att.header.seq=count;
            cmd_att.header.frame_id = 1;
            cmd_att.pose.position.x = 0.0;//0.001*some_object.position_x;
            cmd_att.pose.position.y = 0.0;//0.001*some_object.position_y;
            cmd_att.pose.position.z = 0.0;//0.001*some_object.position_z;
 
            cmd_att.pose.orientation.x = sin(theta/2.0)*v[0]/v_norm;
            cmd_att.pose.orientation.y = sin(theta/2.0)*v[1]/v_norm;
            cmd_att.pose.orientation.z = sin(theta/2.0)*v[2]/v_norm;
            cmd_att.pose.orientation.w = cos(theta/2.0);

            cmd_thr.data = lambda;
       
            pub_att.publish(cmd_att);*/
            /*if((ros::Time::now() - last_request > ros::Duration(0.25)))
            {
                double x = r*cos(phi);
                double y = r*sin(phi);

                pose.pose.position.y=y;
                pose.pose.position.x=x;

                pose.pose.orientation.x = sin(phi/2.0)*v[0]/v_norm;
                pose.pose.orientation.y = sin(phi/2.0)*v[1]/v_norm;
                pose.pose.orientation.z = sin(phi/2.0)*v[2]/v_norm;
                pose.pose.orientation.w = cos(phi/2.0);


                //local_pos_pub.publish(pose);
                phi+=3.14/10;
                //pub_thr.publish(cmd_thr);
                ROS_INFO("x= %lf y= %lf", x, y);
                last_request = ros::Time::now();
            }*/
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}