#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <mavros_msgs/SetMode.h>
#include "ackermann_msgs/AckermannDrive.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/LinkState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"


geometry_msgs::Pose vehicle_pose;
geometry_msgs::PoseWithCovarianceStamped vehicle_stamped_pose; 
ros::Time curr_time;

bool find_index;
int base_link_index; 

double theta = 0;
int r = 3.0;
float w = 1.4;
gazebo_msgs::LinkState vehicle_traj;
ackermann_msgs::AckermannDrive drive_traj;
std::string s = "ackermann_vehicle::base_link";


void circle_trajectory()
{
  theta += w/30.0;

  if(vehicle_pose.position.y < 0.0)
  {
    drive_traj.steering_angle = abs(atan((vehicle_pose.position.y-r*sin(theta))/(vehicle_pose.position.x-r*cos(theta))));
  }
  else
    drive_traj.steering_angle = abs(atan((vehicle_pose.position.x-r*cos(theta))/(vehicle_pose.position.y-r*sin(theta))));

  
  vehicle_pose.position.x = r*cos(theta);
  vehicle_pose.position.y = r*sin(theta);

  vehicle_pose.orientation.w = cos(theta/2.0);
  vehicle_pose.orientation.x = 0;
  vehicle_pose.orientation.y = 0;
  vehicle_pose.orientation.z = sin(theta/2.0);

  curr_time = ros::Time::now();

}

void update_pos(const gazebo_msgs::LinkStates& msg)
{
  if(!find_index)
  {
    for(int i = 0;i<msg.name.size();i++)
    { //ROS_INFO("%d ***********8",msg.name.size());

      if(s.compare(msg.name[i]) == 0)
      {
        base_link_index = i;
        find_index = 1;
        break;
      }
    }
    //ROS_INFO("%d **********",base_link_index);
  }

  vehicle_pose = msg.pose[base_link_index];
  //vehicle_pose = msg.pose[1];
  curr_time = ros::Time::now();
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "ackermann_traj");

  ros::NodeHandle node;

  find_index = 0;
  base_link_index = 0;

  vehicle_pose.position.x = 3.0;
  vehicle_pose.position.y = 0.0;
  vehicle_pose.position.z = 0.0;

  vehicle_traj.link_name = s;

  vehicle_traj.pose = vehicle_pose;

  curr_time = ros::Time::now();

  ros::Publisher drive_pub = node.advertise<ackermann_msgs::AckermannDrive>("/ackermann_cmd", 1000);

  ros::Publisher vehicle_pos_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/ugv/pose", 1000);

  ros::Subscriber vehicle_pose_sub  = node.subscribe("/gazebo/link_states", 1000, update_pos,ros::TransportHints().tcpNoDelay());

  //ros::Publisher vehicle_pose_pub  = node.advertise<gazebo_msgs::LinkState>("/gazebo/set_link_state", 1000);;

  ros::Rate loop_rate(30);


  drive_traj.steering_angle = 0.0;  

  //drive_traj.steering_angle = 0.0316;
  drive_traj.speed = 0.5;

  int frame_sequence_ = 0;

  //ros::Duration time_offset(2.0);

  while (ros::ok())
  {
    //circle_trajectory();

    //drive_traj.speed = r*w;

    drive_pub.publish(drive_traj);

    //vehicle_traj.pose = vehicle_pose;

    //vehicle_pose_pub.publish(vehicle_traj);

    vehicle_stamped_pose.header.stamp = curr_time;

    vehicle_stamped_pose.header.seq = ++ frame_sequence_;

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