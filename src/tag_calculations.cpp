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
//should be standardized at 81x81 mm tags, calibration for macbook pro webcam, yaml at /home/pat/ost.yaml
#define tag_size 0.081
#define fx 696.286438
#define fy 803.754333
#define cx 427.522877
#define cy 312.683933

#include <bu_project/tagdata.h>

//for total over time average
struct average{
  Eigen::Vector3d mDistance;
  double mTheta[3];
  long long polls;
};

average averages[5];
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
    for (int i = 0; i < 3; i++){
      if(theta[di][i]<0)
	theta[di][i]+=2*PI;
      theta[di][i]*=180.0/PI;
    }

  }

  //prints data
  /*for (int i = 0; i < 25; i++)
    std::cout << '-';*/
  for (int i = 0; i < 5; i++){
    if (found[i]&&found[0]&&i!=0){
     /* std::cout << "\nid: " << i << "\n";
      std::cout << "rot: \n" << rot[i] << std::endl;
      std::cout << "trans: \n" << trans[i] << "\n";
      std::cout << "xtheta: " << theta[i][0] << "\n";
      std::cout << "ytheta: " << theta[i][1] << "\n";
      std::cout << "ztheta: " << theta[i][2] << "\n";*/
      /*
      //updates average over time for rotation and translation
      if (found[0]&&i!=0){
	
	//updates average over time for ratation and translation
	averages[i].polls++;
        averages[i].mDistance = averages[i].mDistance*(averages[i].polls-1)/averages[i].polls+(trans[i]-trans[0])/averages[i].polls;
	for (int j = 0; j < 3; j++)
	  averages[i].mTheta[j] = averages[i].mTheta[j]*(averages[i].polls-1)/averages[i].polls+(theta[i][j]-theta[0][j])/averages[i].polls;
	*/
	/*std::cout << "distance from tag 0: \n" << trans[i]-trans[0] << "\n";
	std::cout << "average distance from tag 0: \n" << averages[i].mDistance << "\n";
	std::cout << "xtheta from tag 0: " << theta[i][0]-theta[0][0] << "\n";
	std::cout << "average xtheta from tag 0: " << averages[i].mTheta[0] << "\n";
        std::cout << "ytheta from tag 0: " << theta[i][1]-theta[0][1] << "\n";
	std::cout << "average ytheta from tag 0: " << averages[i].mTheta[1] << "\n";
	std::cout << "ztheta from tag 0: " << theta[i][2]-theta[0][2] << "\n";
        std::cout << "average ztheta from tag 0: " << averages[i].mTheta[2] << "\n";*/
        
        //if tag other than tag 0 is detected as well as tag 0, publish 2d translation and y-axis rotation
	bu_project::tagdata td;
	td.id = i;
	Eigen::Vector3d tempv = trans[i]-trans[0];
	td.x = tempv(0);
	td.y = tempv(1);
	td.theta = theta[i][1]-theta[0][1];
        pub.publish(td);
      //}

      //std::cout << "\n\n";
    }
  }
  //for (int i = 0; i < 25; i++)
  //  std::cout << '-';
  //std::cout << "\n\n\n";

}

int main(int argc, char**argv){
  ros::init(argc, argv, "tag_calculations");
  ros::NodeHandle nh;
  pub = nh.advertise<bu_project::tagdata>("tagdata", 1000);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  ros::spin();
}

