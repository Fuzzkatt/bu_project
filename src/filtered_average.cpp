#include <ros/ros.h>
#include <bu_project/tagdata.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>

const double R = 1.0e-07;
double P[5];
double X[5];
bool firstIteration[5];

void callback(const bu_project::tagdata td){
  for (int i = 0; i < 5; i++){
    double cx = td.data[i].x;
    double cy = td.data[i].y;
    double ct = td.data[i].theta;
    if (cx!=0.0||cy!=0.0||ct!=0.0){
      double Y = sqrt(cx*cx+cy*cy);
      if (firstIteration[i]){
	P[i] = 1;
	X[i] = Y;
	firstIteration[i] = 0;
      }
      else{
        X[i]+=P[i]/(P[i]+R)*(Y-X[i]);
	P[i]*=(1-P[i]/(P[i]+R));
      }
      std::cout << "measured distance of tag " << i << ": " << Y << "\n";
      std::cout << "filtered distance of tag " << i << ": " << X[i] << "\n";
      std::cout << "P of tag " << i << ": " << P[i] << "\n\n";
    }
  }
}

int main(int argc, char**argv){
  for (int i = 0; i < 5; i++)
    firstIteration[i] = 1;
  ros::init(argc, argv, "filtered_average");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("tagdata_raw", 1000, callback);
  ros::spin();
  return 0;
}
