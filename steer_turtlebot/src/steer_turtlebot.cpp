#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt16.h>

#define LEFT  1
#define RIGHT -1
#define TIME_COUNT 40
  

class RobotDriver
{
  int time_count;
  int speed_smoother;
  bool turn_right;
  unsigned long int face_posx;

private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_vel_pub_;
  // Subscribes to /face_detector/face_width to know if a face is detected or not
  ros::Subscriber face_width_sub_;
  ros::Subscriber face_posx_sub_;

public:
  //! ROS node initialization
  RobotDriver() : time_count(0), speed_smoother(0), turn_right(false), face_posx(0)
  {
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);

    // Sets the subscriber for face_width. The size of the queue is 1
    face_width_sub_ = nh_.subscribe("/face_detector/face_width", 1, &RobotDriver::isThereAface, this);
    face_posx_sub_ = nh_.subscribe("/face_detector/face_position_X", 1, &RobotDriver::newPosx, this);
  }

  void moveSlightly()
  {
    int direction;
    geometry_msgs::Twist drive_cmd;

    if (turn_right){
      direction = LEFT;
    }else{
      direction = RIGHT;
    }

      // std::cout<<time_count<<std::endl;

    if (time_count < TIME_COUNT)
    {
      time_count++;
      drive_cmd.angular.z = direction*0.6;
      drive_cmd.linear.x = 0;
      cmd_vel_pub_.publish(drive_cmd);
    }else if (time_count >= TIME_COUNT && time_count < TIME_COUNT+20){
      time_count++;
      drive_cmd.angular.z = 0;
    }else{
      time_count = 0;
      drive_cmd.angular.z = 0;
      turn_right = !turn_right;  
    }
    
  }

  void isThereAface(std_msgs::UInt16 face_width)
  {
    // std::cout<<"face_width: " <<face_width<<std::endl;
    // std::cout<<"face_width.data: "<< face_width.data<<std::endl;
    if (face_width.data == 0)
    {
      // std::cout<<"if"<<std::endl;
      moveSlightly();
    }else{
      moveTowardsFace();
    }
  }

  void newPosx(std_msgs::UInt16 posx){
    face_posx = posx.data;
  }

  void moveTowardsFace(){
    geometry_msgs::Twist drive_cmd;
    if (face_posx < 213){
      drive_cmd.angular.z = LEFT*0.6;
      drive_cmd.linear.x = 0;
    }else if (face_posx > 427){
      drive_cmd.angular.z = RIGHT*0.6;
      drive_cmd.linear.x = 0;
    }else{
      drive_cmd.angular.z = 0;
      drive_cmd.linear.x = 0;
    }
    cmd_vel_pub_.publish(drive_cmd);
  }

};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  RobotDriver robotDriver;
  ros::spin();
  return 0;
}