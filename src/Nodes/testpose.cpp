#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

#include "overworld/Pose.h"

class TestPose
{
public:
  TestPose(double x = 0.0, double y = 0.0, double z = 0.0, double theta = 0.0)
  { 
    cmd_.linear.x = cmd_.linear.y = cmd_.angular.z = 0;
    pose_.theta = theta;
    pose_.x = x;
    pose_.y = y;
    z_ = z;

    ros::NodeHandle n_private("~");

  }
  
  void testLoop();

private:
  
  geometry_msgs::Twist cmd_;
  overworld::Pose pose_;
  double z_;

  ros::NodeHandle n_;
  tf::TransformBroadcaster br_;
  

  void updatePose(double x,double y,double z,double theta);
  void publishTransform();
};

void quit(int sig)
{
  exit(0);
}

int kbhit(void)
{
  struct timeval tv; fd_set
  read_fd; /* Do not wait at all, not even a microsecond */
  tv.tv_sec=0;
  tv.tv_usec=0; /* Must be done first to initialize read_fd */
  FD_ZERO(&read_fd); /* Makes select() ask if input is ready: *
  0 is the file descriptor for stdin */
  FD_SET(0,&read_fd); /* The first parameter is the number of the *
  largest file descriptor to check + 1. */
  if(select(1, &read_fd, NULL, /*No writes*/ NULL, /*No exceptions*/&tv) == -1)
    return 0; /* An error occured */

  /* read_fd now holds a bit map of files that are *
  readable. We test the entry for the standard *
  input (file 0). */
  if(FD_ISSET(0,&read_fd)) /* Character pending on stdin */
    return 1; /* no characters were pending */
  return 0;
}

void TestPose::testLoop()
{
  char c;

  puts("Reading from keyboard");

  double x;
  double y;
  double z;
  double theta;

  for(;;)
  {
    if(kbhit())
    {
      std::cout<<"Please enter the coordinate of the agent:" <<std::endl;
      std::cout<<"x:";
      std::cin>>x;
      publishTransform();
      std::cout<<"y:";
      std::cin>>y;
      publishTransform();
      std::cout<<"z:";
      std::cin>>z;
      publishTransform();
      std::cout<<"theta:";
      std::cin>>theta;

      updatePose(x,y,z,theta);
    }
    else
      usleep(10000);
    
    publishTransform();
  }
}

void TestPose::updatePose(double x,double y,double z,double theta)
{
  pose_.theta = theta;
  pose_.x =x;
  pose_.y =y;
  z_=z;
}

void TestPose::publishTransform()
{
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(pose_.x, pose_.y, z_) );
  tf::Quaternion q;
  q.setRPY(0, 0, pose_.theta);
  transform.setRotation(q);
  br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "overworld_testpose");

  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double theta = 0.0;

  TestPose testpose(x, y, z, theta);

  signal(SIGINT,quit);

  testpose.testLoop();

  return 0;
}
