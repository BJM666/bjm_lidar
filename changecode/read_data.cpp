#include <iostream>

#include <Eigen/Dense>
#include <math.h>
#define roll 2.99
#define pitch 8.34853
#define yaw -90.9782
#define PI 3.1415926
using namespace Eigen;

int main()

{
  Matrix3f x,y,z;

  x << 1, 0, 0,
       0, cos(roll/180*PI),-sin(roll/180*PI),
       0, sin(roll/180*PI), cos(roll/180*PI);

  y <<cos(pitch/180*PI),0,sin(pitch/180*PI),
      0,    1,     0,
      -sin(pitch/180*PI),0,cos(pitch/180*PI);

  z <<cos(yaw/180*PI),-sin(yaw),0,
      sin(yaw/180*PI),cos(yaw/180*PI),0,
      0,0,1;
  std::cout <<z*y*x << std::endl;

}
