#include <ros/ros.h>
#include <bu_project/tagdata3d.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <cmath>
#include <std_msgs/Float64.h>
//#include <bu_project/filteredaverage.h>
#include <Eigen/Dense>

#define PI 3.14159265

//R and Q here are just for rotation; should have a different R and Q for tag to tag mapping
const double R = 1.0e-06;
const double Q = 1.0e-06; //temp for now; need to measure through sampling later

//these are all just for rotation right now
typedef Eigen::Matrix<double, 5, 5> Matrix5d;
typedef Eigen::Matrix<double, 5, 1> Vector5d;
Matrix5d P;
Vector5d X;
bool firstIteration = 1;

ros::Publisher pub;

void callback(const bu_project::tagdata3d td){
  double x0 = td.points[0].x;
  double y0 = td.points[0].y;
  double z0 = td.points[0].z;
  double Y = -td.angles[0];
  
  if (x0!=0.0||y0!=0.0||z0!=0.0||Y!=0.0){
    if (firstIteration){
      firstIteration = 0;
      //initialize X
      X(0) = Y;
      for (int i = 1; i < 5; i++){
	double yaw = td.angles[i];
	if (yaw!=0.0)
	  X(i) = yaw;
      }
      //initialize P
      for (int i = 0; i < 5; i++){
	for (int j = 0; j < 5; j++){
	  if (i==j)
	    P(i, j) = R;
	  else
	    P(i, j) = 0;
	}
      }
    }
    else{
    }
  }

}

void callback2(const std_msgs::Float64 f){
  if(!firstIteration){
    Vector5d v;
    v(0) = f.data;
    for (int i = 1; i < 5; i++)
      v(i) = 0;
    Matrix5d m;
    m(0, 0) = Q;
    for (int i = 0; i < 5; i++){
      for (int j = 0; j < 5; j++){
	if (!(i==0&&j==0))
	  m(i, j) = 0;
      }
    }

    X+=v;
    P+=m;
  }

}



int main (int argc, char** argv){
  ros::init(argc, argv, "robot_pose");
  ros::NodeHandle nh;
  pub = nh.advertise<std_msgs::Float64>("P_value", 1000);
  ros::Subscriber sub = nh.subscribe("tagdata_raw", 1000, callback);
  ros::Subscriber sub2 = nh.subscribe("yaw_values", 1000, callback2);
  ros::spin();
  return 0;
}
