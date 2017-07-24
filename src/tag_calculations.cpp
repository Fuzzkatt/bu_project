#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <apriltags_ros/apriltag_detector.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/foreach.hpp>
#include <AprilTags/Tag36h11.h>

#include <math.h>

#define PI 3.14159265
//should be standardized at 80x80 mm tags, calibration for macbook pro webcam, yaml at /home/pat/ost.yaml
#define tag_size 0.080
#define fx 696.286438
#define fy 803.754333
#define cx 427.522877
#define cy 312.683933

#include <bu_project/tagdata.h>
#include <geometry_msgs/Pose2D.h>

ros::Publisher pub;

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

  //extracts rotation and translation matrices, marks tags as found, converts rotation to 0 to 360 angle in degrees
  BOOST_FOREACH(AprilTags::TagDetection detection, detections){
    int di = detection.id;
    found[di] = 1; 
    detection.getRelativeTranslationRotation(tag_size, fx, fy, cx, cy, trans[di], rot[di]);
    theta[di][0] = atan2(rot[di](1, 2), rot[di](2, 2));
    theta[di][1] = atan2(-rot[di](0, 2), sqrt(rot[di](1, 2)*rot[di](1, 2) + rot[di](2, 2)*rot[di](2, 2)));
    theta[di][2] = atan2(rot[di](0, 1), rot[di](0, 0));
  }

  bu_project::tagdata td;
  for (int i = 0; i < 5; i++){
    if (found[i]){
      //push pose2d data in case of detection
      geometry_msgs::Pose2D temp;
      temp.x = trans[i](0);
      temp.y = trans[i](1);
      temp.theta = theta[i][1];
      td.data.push_back(temp);
    }
    else{
      //otherwise, pose blank msg representing no data collected for tag i
      geometry_msgs::Pose2D blank;
      blank.x = 0.0;
      blank.y = 0.0; 
      blank.theta = 0.0;
      td.data.push_back(blank);
    }
  }

  pub.publish(td);
}

int main(int argc, char**argv){
  ros::init(argc, argv, "tag_calculations");
  ros::NodeHandle nh;
  pub = nh.advertise<bu_project::tagdata>("tagdata_raw", 1000);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  ros::spin();
}

