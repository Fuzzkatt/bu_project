#include <ros/ros.h>
#include <bu_project/tagdata3d.h>
#define PI 3.14159265

int polls = 0;
double measurement[100];

void callback(const bu_project::tagdata3d td){
  double x = td.points[0].x;
  double y = td.points[0].y;
  double z = td.points[0].z;
  double t = -td.angles[0];

  if (x!=0.0||y!=0.0||z!=0.0||t!=0.0){
    if (polls==100){
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
      measurement[polls]=t;
      std::cout << "poll: " << polls+1 << " | measurement: " << measurement[polls] << "\n";
      polls++;
    }
  }

}

int main(int argc, char** argv){
  ros::init(argc, argv, "gyro_noise_test");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("tagdata_raw", 1000, callback);
  ros::spin();
  return 0;
}
