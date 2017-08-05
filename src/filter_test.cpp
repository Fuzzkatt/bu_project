#include <ros/ros.h>
#include <iostream>
#include <random>
#include <algorithm>
#include <time.h>
#include <math.h>
#include <std_msgs/Float64.h>

#define N 100

const double R = 2.0;
double fa = 0.0;
double P = 2.0;
double sa = 0.0;
double M, S;
double data[N];
ros::Publisher pub;

int main(int argc, char**argv){
  ros::init(argc, argv, "filter_test");
  ros::NodeHandle nh;
  pub = nh.advertise<std_msgs::Float64>("generated_value", 1000);

  std::cin >> M;
  S = time(NULL);

  std::default_random_engine g;
  g.seed(S);
  std::normal_distribution<double> d(M, 2.0);

  for (int i = 0; i < N; i++){
    data[i] = d(g);
    if (i == 0)
      fa = data[i];
    else{
      fa+=P/(P+R)*(data[i]-fa);
      P*=(1-P/(P+R));
    }
    sa+=data[i];
  }
  sa/=N;

  std::sort (data, data+N);
  for (int i = 0; i < N; i++){
    std_msgs::Float64 f;
    f.data = data[i];
    pub.publish(f);
  }

  std::cout << "filtered average: " << fa << "\n";
  std::cout << "sum average: " << sa << "\n";
   

  ros::shutdown();
  
}
