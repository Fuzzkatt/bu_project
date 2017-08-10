#include <ros/ros.h>
#include <std_msgs/Float64.h>
#define PI 3.14158265

int polls;
double measurement[100];

void callback(const std_msgs::Float64 f){
  double m = f.data;
  if (polls == 100){
    double mean = 0;
    for (int i = 0; i < 100; i++)
      mean+=measurement[i];
    mean/=100.0;
    double variance = 0;
    for (int i = 0; i < 100; i++)
      variance+=(measurement[i]-mean)*(measurement[i]-mean);
    variance/=100.0;
    std::cout << "mean: " << mean << "\n";
    std::cout << "variance: " << variance << "\n";
    ros::shutdown();
  }
  else{
    measurement[polls]=m;
    std::cout << "poll: " << polls+1 << " | measurement: " << measurement[polls] << "\n";
    polls++;
  }
}


int main(int argc, char**argv){
  ros::init(argc, argv, "rotation_noise_test");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("yaw_values", 1000, callback);
  ros::spin();
  return 0;
}
