#include "ros/ros.h"
#include "Eigen/Dense"
#include <math.h>

#define Pi 3.1415926

using namespace Eigen;

void rotate(float yaw,  const Vector3f& input, Vector3f& output);

Vector3f local_pos(1.0,1.0,0.0);
Vector3f body_pos(0.0,0.0,0.0);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle n;
  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    rotate(Pi/4, local_pos, body_pos);
    ROS_INFO("\nx:%f\ny:%f\nz:%f",body_pos(0),body_pos(1),body_pos(2));

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void rotate(float yaw,  const Vector3f& input,  Vector3f& output)
{
  float sy = sinf(yaw);
  float cy = cosf(yaw);

  Matrix3f data;
  data(0,0) = cy;
  data(0,1) = -sy;
  data(0,2) = 0.0;
  data(1,0) = sy;
  data(1,1) = cy;
  data(1,2) = 0.0;
  data(2,0) = 0.0;
  data(2,1) = 0.0;
  data(2,2) = 1.0;

  output = data * input;
  ROS_INFO("\nox:%f\noy:%f\noz:%f",output(0),output(1),output(2));
}
