#include <ros/ros.h>
#include <bu_project/tagdata3d.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <cmath>
#include <std_msgs/Float64.h>
//#include <bu_project/filteredaverage.h>
#include <Eigen/Dense>

#define PI 3.14159265

const double R = 1.0e-07; 
const double Q = 1.0e-07; //temp for now; need to measure through sampling later
double P;
double X, Xh;
bool firstIteration = 1;

ros::Publisher pub;

void callback(const bu_project::tagdata3d td){
  double x0 = td.points[0].x;
  double y0 = td.points[0].y;
  double z0 = td.points[0].z;
  double Y = -td.angles[0];

  if (x0!=0.0||y0!=0.0||z0!=0.0||Y!=0.0){ 
    if (firstIteration){
      P = Q;
      X = Y;
      Xh = Y;
      firstIteration = 0;
    }
    else{
      P+=Q;
      Xh=X+P/(P+R)*(Y-X);
      P*=(1-P/(P+R)); 
      X=Xh;
    }

    Eigen::Matrix2d rot;
    rot(0, 0) = cos(Y);
    rot(1, 0) = -sin(Y);
    rot(0, 1) = sin(Y);
    rot(1, 1) = cos(Y);

    Eigen::Vector2d trans(x0, y0);
    trans = rot*trans;
    double distance = sqrt(trans(0)*trans(0) + trans(1)*trans(1));

    //std::cout << "measured translation of robot with respect to tag 0\n" << trans << "\n";
    //std::cout << "measured rotation vector of robot with respect to tag 0\n" << rot << "\n";

    std::cout << "measured distance of robot with respect to tag 0: " << distance << " m @ " << Xh/PI*180.0 << " degrees\n";
  }
}

void callback2(const std_msgs::Float64 f){
  if (!firstIteration){
    X+= f.data*PI/180.0; 
  //outputs change in theta from original position
  //always start directly facing tag
  //might need to multiply by -1 since robot to tag theta is 180 rotation from way robot is facing?  
  }
}

int main (int argc, char** argv){
  ros::init(argc, argv, "robot_pose");
  ros::NodeHandle nh;
  pub = nh.advertise<bu_project::tagdata3d>("robot_pose", 1000);
  ros::Subscriber sub = nh.subscribe("tagdata_raw", 1000, callback);
  ros::Subscriber sub2 = nh.subscribe("yaw_values", 1000, callback2);
  ros::spin();
  return 0;
}
