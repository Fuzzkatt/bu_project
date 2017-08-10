#include <ros/ros.h>
#include <bu_project/tagdata3d.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <cmath>
#include <std_msgs/Float64.h>
//#include <bu_project/filteredaverage.h>
#include <Eigen/Dense>

#define PI 3.14159265

const double R = 1.0e-06; //vision 
const double Q = 2.5e-06; //gyro
double P;
double X;
bool firstIteration = 1;

ros::Publisher pub;

void callback(const bu_project::tagdata3d td){
  double x0 = td.points[0].x;
  double y0 = td.points[0].y;
  double z0 = td.points[0].z;
  double Y = -td.angles[0];

  if (x0!=0.0||y0!=0.0||z0!=0.0||Y!=0.0){ 
    if (firstIteration){
      P = R;
      X = Y;
      firstIteration = 0;
    }
    else{
      X+=P/(P+R)*(Y-X);
      P*=(1-P/(P+R));
    }
    
    std_msgs::Float64 temp;
    temp.data = X*180.0/PI;
    pub.publish(temp);
    Eigen::Matrix2d rot;
    rot(0, 0) = cos(X);
    rot(1, 0) = -sin(X);
    rot(0, 1) = sin(X);
    rot(1, 1) = cos(X);

    Eigen::Vector2d trans(x0, y0);
    trans = rot*trans;
    double distance = sqrt(trans(0)*trans(0) + trans(1)*trans(1));

    //std::cout << "measured translation of robot with respect to tag 0\n" << trans << "\n";
    //std::cout << "measured rotation vector of robot with respect to tag 0\n" << rot << "\n";

    std::cout << "y: " << Y/PI*180.0 << "\n";
    std::cout << "filtered distance of robot with respect to tag 0: " << distance << " m @ " << X/PI*180.0 << " degrees\n";
  }
}

void callback2(const std_msgs::Float64 f){
  if (!firstIteration){
    X+=f.data*PI/180.0;
    P+=Q;
    std::cout << "filtered angle  of robot with respect to tag 0: " << X/PI*180.0 << " degrees\n"; 
    std_msgs::Float64 temp;
    temp.data = X*180.0/PI;
    pub.publish(temp);
  //outputs change in theta from original position
  //always start directly facing tag
  //might need to multiply by -1 since robot to tag theta is 180 rotation from way robot is facing?  
  }
}

int main (int argc, char** argv){
  ros::init(argc, argv, "robot_pose");
  ros::NodeHandle nh;
  pub = nh.advertise<std_msgs::Float64>("filtered_angle", 1000);
  ros::Subscriber sub = nh.subscribe("tagdata_raw", 1000, callback);
  ros::Subscriber sub2 = nh.subscribe("yaw_values", 1000, callback2);
  ros::spin();
  return 0;
}
