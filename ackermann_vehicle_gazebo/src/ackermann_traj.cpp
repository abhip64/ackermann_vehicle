////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CODE FOR MOVING THE ACKERMANN VEHICLE AT A GIVEN STEERING ANGLE AND VELOCITY. THE POSITION OF THE BODY LINK
//IS OBTAINED FROM GAZEBO AND THEN REPUBLISHED. THE REPUBLISHED POSITION DATA IS USED BY A KALMAN FILTER
//TO PREDICT THE POSITION, VELOCITY, ACCELERATION AND ATTITUDE OF THE VEHCILE
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//HEADER FILES

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <mavros_msgs/SetMode.h>
#include "ackermann_msgs/AckermannDrive.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/LinkState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

geometry_msgs::Pose vehicle_pose;
geometry_msgs::PoseWithCovarianceStamped vehicle_stamped_pose; 
ros::Time curr_time;

bool find_index;
int base_link_index; 

ackermann_msgs::AckermannDrive drive_traj;

//Name of the link from which the position data of the ackermann vehicle is obtained from Gazebo
std::string s = "ackermann_vehicle::base_link";

//Function to obtain the position of the ackermann vehicle. The position is obtained by subscribing
//to the link positions published by Gazebo. The position of the ackermann vehicle is taken as the 
//link position of its base link.
void update_pos(const gazebo_msgs::LinkStates& msg)
{
  if(!find_index)
  {
    for(int i = 0;i<msg.name.size();i++)
    { 
      if(s.compare(msg.name[i]) == 0)
      {
        base_link_index = i;
        find_index = 1;
        break;
      }
    }
  }
  vehicle_pose = msg.pose[base_link_index];
  curr_time = ros::Time::now();
}


int main(int argc, char **argv)
{

  //Initialise ROS node with the name "ackermann_traj"
  ros::init(argc, argv, "ackermann_traj");

  ros::NodeHandle node;

  find_index = 0;
  base_link_index = 0;

  //Intial Position of the vehicle
  vehicle_pose.position.x = 0.0;
  vehicle_pose.position.y = 0.0;
  vehicle_pose.position.z = 0.0;

  curr_time = ros::Time::now();

  //Publisher for publishing the ackermann vehicle steering angle and speed for controlling the vehicle
  ros::Publisher drive_pub          = node.advertise<ackermann_msgs::AckermannDrive>("/ackermann_cmd", 1000);

  //Publisher for publishing the position of the vehicle after adding required amount of noise. This format
  //of publishing is required for the robot_localisation package 
  ros::Publisher vehicle_pos_pub    = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/ugv/pose", 1000);

  //Subscriber for getting the position data of the base link of the ackermann vehicle
  ros::Subscriber vehicle_pose_sub  = node.subscribe("/gazebo/link_states", 1000, update_pos,ros::TransportHints().tcpNoDelay());

  //Publish rate of the vehicle position data
  ros::Rate loop_rate(30);

  //Steering angle of the vehicle
  drive_traj.steering_angle = 0.0;  

  //Speed of travel of the vehicle
  drive_traj.speed = 0.5;

  int frame_sequence_ = 0;

  while (ros::ok())
  {

    //Publish control data of ackermann vehicle
    drive_pub.publish(drive_traj);

    //Publish the current position of the vehicle by adding co variances

    vehicle_stamped_pose.header.stamp = curr_time;

    vehicle_stamped_pose.header.seq = ++ frame_sequence_;

    //Required format for robot_localisation package
    vehicle_stamped_pose.header.frame_id = "odom";

    vehicle_stamped_pose.pose.pose = vehicle_pose;

    vehicle_stamped_pose.pose.covariance = boost::array<double, 36>({
          0., 0., 0., 0., 0., 0.,
          0., 0., 0., 0., 0., 0.,
          0., 0., 0., 0., 0., 0.,
          0., 0., 0., 0., 0., 0.,
          0., 0., 0., 0., 0., 0.,
          0., 0., 0., 0., 0., 0.
        });

    vehicle_pos_pub.publish(vehicle_stamped_pose);

    ros::spinOnce();

    loop_rate.sleep();
    
  }
  return 0;
}