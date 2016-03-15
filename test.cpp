#include "ros/ros.h"
#include "Eigen/Dense"
#include <math.h>

#include "std_msgs/String.h"
#include <sstream>

#define Pi 3.1415926

using namespace Eigen;

void rotate(float roll, float pitch, float yaw, Vector3f input, Vector3f output);

Vector3f local_pos(1.0,1.0,0.0);
Vector3f body_pos(0.0,0.0,0.0);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    rotate(0.0, 0.0, Pi/2, local_pos, body_pos);
    ROS_INFO("x:%f\ny:%f\nz:%f",body_pos(0),body_pos(1),body_pos(2));

    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    chatter_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void rotate(float roll, float pitch, float yaw, Vector3f input, Vector3f output) 
{
  float cp = cosf(pitch);
  float sp = sinf(pitch);
  float sr = sinf(roll);
  float cr = cosf(roll);
  float sy = sinf(yaw);
  float cy = cosf(yaw);

  Matrix3f data;
  data[0][0] = cp * cy;
  data[0][1] = (sr * sp * cy) - (cr * sy);
  data[0][2] = (cr * sp * cy) + (sr * sy);
  data[1][0] = cp * sy;
  data[1][1] = (sr * sp * sy) + (cr * cy);
  data[1][2] = (cr * sp * sy) - (sr * cy);
  data[2][0] = -sp;
  data[2][1] = sr * cp;
  data[2][2] = cr * cp;

  output = data * input;
}
