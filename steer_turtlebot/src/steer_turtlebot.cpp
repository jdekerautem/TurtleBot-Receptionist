#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt16.h>

#define LEFT  1
#define RIGHT -1
  

class RobotDriver
{
  int time_count;
  int speed_smoother;
  bool turn_right;

private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_vel_pub_;
  // Subscribes to /face_detector/face_width to know if a face is detected or not
  ros::Subscriber face_width_sub_;

public:
  //! ROS node initialization
  RobotDriver() : time_count(0), speed_smoother(0), turn_right(false)
  {
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);

    // Sets the subscriber for face_width. The size of the queue is 1
    face_width_sub_ = nh_.subscribe("/face_detector/face_width", 1, &RobotDriver::isThereAface, this);
  }

  //! Loop forever while sending drive commands based on keyboard input
  // bool driveKeyboard()
  // {
  //   std::cout << "Type a command and then press enter.  "
  //     "Use '+' to move forward, 'l' to turn left, "
  //     "'r' to turn right, '.' to exit.\n";

  //   //we will be sending commands of type "twist"
  //   geometry_msgs::Twist base_cmd;

  //   char cmd[50];
  //   while(nh_.ok()){

  //     std::cin.getline(cmd, 50);
  //     if(cmd[0]!='+' && cmd[0]!='l' && cmd[0]!='r' && cmd[0]!='.')
  //     {
  //       std::cout << "unknown command:" << cmd << "\n";
  //       continue;
  //     }

  //     base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
  //     //move forward
  //     if(cmd[0]=='+'){
  //       base_cmd.linear.x = 0.25;
  //     }
  //     //turn left (yaw) and drive forward at the same time
  //     else if(cmd[0]=='l'){
  //       base_cmd.angular.z = 0.75;
  //       base_cmd.linear.x = 0.25;
  //     }
  //     //turn right (yaw) and drive forward at the same time
  //     else if(cmd[0]=='r'){
  //       base_cmd.angular.z = -0.75;
  //       base_cmd.linear.x = 0.25;
  //     }
  //     //quit
  //     else if(cmd[0]=='.'){
  //       break;
  //     }

  //     //publish the assembled command
  //     cmd_vel_pub_.publish(base_cmd);
  //   }
  //   return true;
  // }

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

    if (time_count < 30)
    {
      time_count++;
      drive_cmd.angular.z = direction*0.6;
      drive_cmd.linear.x = 0;
      cmd_vel_pub_.publish(drive_cmd);
    }else if (time_count >= 30 && time_count < 35){
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
    }
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