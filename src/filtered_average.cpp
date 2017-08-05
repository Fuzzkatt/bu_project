#include <ros/ros.h>
#include <bu_project/tagdata3d.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <cmath>
#include <bu_project/filteredaverage.h>
#include <Eigen/Dense>

const double R = 1.0e-07;
double P[5];
double X[5];
bool firstIteration[5];
ros::Publisher pub;
ros::Publisher pub2;

void callback(const bu_project::tagdata3d td){
  bu_project::filteredaverage fa;
  bu_project::filteredaverage md;

  double x0 = td.points[0].x;
  double y0 = td.points[0].y;
  double z0 = td.points[0].z;
  double t = -td.angles[0];

  Eigen::Matrix2d rot;
  rot(0, 0) = cos(t);
  rot(1, 0) = -sin(t);
  rot(0, 1) = sin(t);
  rot(1, 1) = cos(t);

  std_msgs::Float64 f;
  f.data = 0;
  fa.averages.push_back(f);
  f.data = 0;
  md.averages.push_back(f);

  if (x0!=0.0||y0!=0.0||z0!=0.0||t!=0.0){
    for (int i = 1; i < 5; i++){
      double cx = td.points[i].x;
      double cy = td.points[i].y;
      double cz = td.points[i].z;
      double ct = td.angles[i];

      Eigen::Vector2d trans0(x0, y0);
      Eigen::Vector2d trans1(cx, cy);
      Eigen::Vector2d distancev = rot*(trans1-trans0);
      
      double Y = sqrt(distancev(0)*distancev(0) + distancev(1)*distancev(1));
      //double Y = sqrt(distancev(2)*distancev(2) + temp*temp);

      if (cx!=0.0||cy!=0.0||cz!=0.0||ct!=0.0){
        if (firstIteration[i]){
	  X[i] = Y;
	  P[i] = R;
	  firstIteration[i] = 0;
        }
        else{
          X[i]+=P[i]/(P[i]+R)*(Y-X[i]);
	  P[i]*=(1-P[i]/(P[i]+R));
        }
      //std::cout << "measured position of tag " << i << ": with respect to tag 0:\n" << distancev << "\n";
      //std::cout << "filtered distance of tag " << i << ": " << X[i] << "\n";
      //std::cout << "P of tag " << i << ": " << P[i] << "\n\n";
      }

      std_msgs::Float64 f;
      f.data = X[i];
      fa.averages.push_back(f);
      f.data = Y;
      md.averages.push_back(f);
    }
    pub.publish(fa);
    pub2.publish(md);
  }
}

int main(int argc, char**argv){
  for (int i = 0; i < 5; i++){
    firstIteration[i] = 1;
    X[i] = nan("");
    P[i] = nan("");
  }
  ros::init(argc, argv, "filtered_average");
  ros::NodeHandle nh;
  pub = nh.advertise<bu_project::filteredaverage>("filtered_average", 1000);
  pub2 = nh.advertise<bu_project::filteredaverage>("measured_distance", 1000);
  ros::Subscriber sub = nh.subscribe("tagdata_raw", 1000, callback);
  ros::spin();
  return 0;
}
