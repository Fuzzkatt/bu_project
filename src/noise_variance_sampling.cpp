#include <ros/ros.h>
#include <bu_project/tagdata3d.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>

int polls[5];
double measurement[5][100];

void callback(const bu_project::tagdata3d td){
  //will update later
  /*for (int i = 0; i < 5; i++){
    double cx = td.data[i].x;
    double cy = td.data[i].y;
    double ct = td.data[i].theta;
    if (cx!=0.0||cy!=0.0||ct!=0.0){
      if (polls[i]==100){
        double mean = 0;
	for (int j = 0; j < 100; j++)
	  mean+=measurement[i][j];
	mean/=100.0;
	double variance = 0;
	for (int j = 0; j < 100; j++)
	  variance+=(measurement[i][j]-mean)*(measurement[i][j]-mean);
	variance/=100.0;
	std::cout << "mean: " << mean <<"\n";
	std::cout << "variance: " << variance << "\n";
	ros::shutdown();
      }
      else{
        measurement[i][polls[i]] = sqrt(cx*cx+cy*cy);
	std::cout << "poll: " << polls[i] << " distance: " << measurement[i][polls[i]] << " x: " << cx << " y: " << cy << "\n";
        polls[i]++;
      }
    }
  }*/
}

int main (int argc, char**argv){
  for (int i = 0; i < 5; i++){
    polls[i] = 0;
    for (int j = 0; j < 100; j++)
      measurement[i][j] = 0;
  }
  ros::init(argc, argv, "noise_variance_sampling");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("tagdata_raw", 1000, callback);
  ros::spin();
  return 0;
}
