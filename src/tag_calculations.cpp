#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <apriltags_ros/apriltag_detector.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include </home/pat/it_ws/devel/.private/apriltags_ros/include/apriltags_ros/AprilTagDetection.h>
#include </home/pat/it_ws/devel/.private/apriltags_ros/include/apriltags_ros/AprilTagDetectionArray.h>
#include <AprilTags/Tag36h11.h>

#include <math.h>

#define PI 3.14159265
//should be standardized at 81x81 mm tags, calibration for macbook pro webcam, yaml at /home/pat/ost.yaml
#define tag_size 0.081
#define fx 696.286438
#define fy 803.754333
#define cx 427.522877
#define cy 312.683933

//for total over time average
struct average{
  Eigen::Vector3d mean;
  long long polls;
};

average aDistance[5];

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat gray;
  cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
  const AprilTags::TagCodes* tag_codes = &AprilTags::tagCodes36h11;
  boost::shared_ptr<AprilTags::TagDetector> tag_detector_= boost::shared_ptr<AprilTags::TagDetector>(new AprilTags::TagDetector(*tag_codes));
  std::vector<AprilTags::TagDetection> detections = tag_detector_->extractTags(gray);
  
  Eigen::Vector3d trans [5];
  Eigen::Matrix3d rot [5];
  Eigen::Vector3d distance[5]; //distance[i] = distance from tag i to tag 0;
  bool found[5]; //tracks if tag detected in current callback
  double theta[5][3]; //col 0 x, col 1 y, col 2 z

  for (int i = 0; i < 5; i++)
    found[i] = 0;

  //extracts data from tag detection
  BOOST_FOREACH(AprilTags::TagDetection detection, detections){
    int di = detection.id;
    found[di] = 1; 
    detection.getRelativeTranslationRotation(tag_size, fx, fy, cx, cy, trans[di], rot[di]);
    theta[di][0] = atan2(rot[di](1, 2), rot[di](2, 2));
    theta[di][1] = atan2(-rot[di](0, 2), sqrt(rot[di](1, 2)*rot[di](1, 2) + rot[di](2, 2)*rot[di](2, 2)));
    theta[di][2] = atan2(rot[di](0, 1), rot[di](0, 0));
  }

  //prints data
  for (int i = 0; i < 25; i++)
    std::cout << '-';
  std::cout << "\ntest";
  for (int i = 0; i < 5; i++){
    if (found[i]){
      std::cout << "\nid: " << i << "\n";
      std::cout << "rot: \n" << rot[i] << std::endl;
      std::cout << "trans: \n" << trans[i] << "\n";
      std::cout << "xtheta: " << theta[i][0]*180.0/PI << "\n";
      std::cout << "ytheta: " << theta[i][1]*180.0/PI << "\n";
      std::cout << "ztheta: " << theta[i][2]*180.0/PI << "\n";

      if (found[0]&&i!=0){
	aDistance[i].polls++;
	if (aDistance[i].polls == 1)
	  aDistance[i].mean = trans[i]-trans[0];
	else
          aDistance[i].mean = aDistance[i].mean*(aDistance[i].polls-1)/aDistance[i].polls+(trans[i]-trans[0])/aDistance[i].polls;
	std::cout << "distance from tag 0: " << trans[i]-trans[0] << "\n";
	std::cout << "average distane from tag 0: " << aDistance[i].mean << "\n";
      }

      std::cout << "\n\n";
    }
  }
  for (int i = 0; i < 25; i++)
    std::cout << '-';
  std::cout << "\n\n\n";

}

int main(int argc, char**argv){
  ros::init(argc, argv, "tag_calculations");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  ros::spin();
}

