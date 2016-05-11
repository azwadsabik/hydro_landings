#include <string>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Core>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <std_msgs/Bool.h>

using namespace std;

string quadrotor_base;
geometry_msgs::Pose goal;
std_msgs::Bool goal_reached;
bool goal_assigned = false;
bool alive = true;
Eigen::Matrix3d P = Eigen::Vector3d(0.1,0.1,0.3).asDiagonal();
Eigen::Matrix3d I = Eigen::Vector3d(0,0,0).asDiagonal();
Eigen::Matrix3d D = Eigen::Vector3d(0.5,0.5,0.5).asDiagonal();
float P_x = 0.5;
float P_y = 0.5;
float P_z = 0.8;
float e_max = 0.1;
Eigen::Vector3d error_sum(0,0,0);
Eigen::Vector3d error_prev(0,0,0);
Eigen::Vector3d error_eigen(0,0,0);
Eigen::Vector3d error_dev(0,0,0);

geometry_msgs::Twist calculateVelocity(const tf::StampedTransform& robot_in_world)
{
  geometry_msgs::Twist velocity;
  tf::Matrix3x3 world_in_robot_rotation = robot_in_world.getBasis().inverse();
  tf::Vector3 robot_in_world_position = robot_in_world.getOrigin();  
  tf::Vector3 goal_in_world_position(goal.position.x, goal.position.y, goal.position.z);
 
  cout << "Location of robot is ";
  cout << robot_in_world_position.getX() << ", ";
  cout << robot_in_world_position.getY() << ", ";
  cout << robot_in_world_position.getZ() << endl;
  cout << "Location of goal is ";
  cout << goal_in_world_position.getX() << ", ";
  cout << goal_in_world_position.getY() << ", ";
  cout << goal_in_world_position.getZ() << endl;  

  tf::Vector3 error = goal_in_world_position - robot_in_world_position;

  tf::vectorTFToEigen(error, error_eigen);
  float e = error.length();

  cout << "Distance to target is " << e << endl;

  error_sum += error_eigen;
  error_dev = error_eigen - error_prev;

  Eigen::Vector3d velocity_in_world_eig(P*error_eigen+I*error_sum+D*error_dev);
  tf::Vector3 velocity_in_world; 
  //tf::Vector3 velocity_in_world(P_x*error.x(), P_y*error.y(), P_z*error.z());
  tf::vectorEigenToTF(velocity_in_world_eig,velocity_in_world);
  tf::Vector3 velocity_in_robot = world_in_robot_rotation*velocity_in_world;

  error_prev = error_eigen;
  
  velocity.linear.x = velocity_in_robot.x();
  velocity.linear.y = velocity_in_robot.y();
  velocity.linear.z = velocity_in_robot.z();
  velocity.angular.x = 0;
  velocity.angular.y = 0;
  velocity.angular.z = 0;

  if(e < e_max){  
    goal_reached.data = true;
  }
  else{
    goal_reached.data = false;
  }
  
  return velocity;
}

bool unequal_poses(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b){
  bool unequal = false;
  unequal = (a.position.x != b.position.x);
  unequal = (a.position.y != b.position.y);
  unequal = (a.position.z != b.position.z);
  return unequal;
}

void goalCallBack(const geometry_msgs::Pose& new_goal)
{
  goal_assigned = true;
  if (unequal_poses(goal, new_goal)){
	  cout << "Goal was updated!" << endl;
	  goal_reached.data = false;
  }
  goal = new_goal;
}

void killCallBack(const std_msgs::Bool& kill)
{
	if (kill.data){
	cout << "Killing Node!" << endl;
		ros::shutdown();
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_cmd");
  
  ros::NodeHandle node;
  
  if (node.getParam("/quadrotor_base", quadrotor_base)){
  } else {
      quadrotor_base = "/ardrone_base_link";
  }

  tf::TransformListener robot_pose_estimator;
  ros::Subscriber goal_sub = node.subscribe("goal", 1000, goalCallBack); 
  ros::Subscriber alive = node.subscribe("/kill_velocity", 1000, killCallBack);
  ros::Publisher velocity_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1000); 
  ros::Publisher goal_reached_pub = node.advertise<std_msgs::Bool>("goal_reached", 1000); 

  ros::Rate rate(50.0);

  while(node.ok()){
    tf::StampedTransform robot_in_world;
    try{
      robot_pose_estimator.lookupTransform("/world",quadrotor_base,ros::Time(),robot_in_world);
      cout << "Detected Transform" << endl;
      if (!goal_assigned){
	    tf::Vector3 robot_in_world_position = robot_in_world.getOrigin(); 
		goal.position.z = 1.5;
		goal.position.x = robot_in_world_position.getX();
		goal.position.y = robot_in_world_position.getY();
	  }
	  geometry_msgs::Twist command_velocity = calculateVelocity(robot_in_world);
	  velocity_pub.publish(command_velocity);
      goal_reached_pub.publish(goal_reached);
    }
    catch(tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      cout << "No Transform Found" << endl;
    }

	ros::spinOnce();
    rate.sleep();
  }
  return 0;
};
