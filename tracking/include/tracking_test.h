#ifndef MUL_TRACKING
#define MUL_TRACKING

#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <string>
#include <vector>
#include "iarc_msgs/RoiPos.h"
#include "include/kcftracker.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "std_msgs/Int8.h"

#define HOG 1
#define FIXEDWINDOW 1
#define MULTISCALE 1
#define LAB 1
#define SRC_WINDOW_NAME "redball"
#define MID_WINDOWNAME "redball_gray"

typedef std::chrono::time_point<std::chrono::high_resolution_clock> timer;
typedef std::chrono::duration<float> duration;

using namespace std;
using namespace cv;

class my_target {
 public:
  Mat pic;
  Rect bbox;
  int num;
  int match;
  Scalar my_color;
  vector<Point> my_history;
  double match_sim(vector<my_target> old);
  double getMSSIM(Mat inputimage1, Mat inputimage2);
};

class Tracking_Melon {
 public:
  Mat HSV, mask, rode, dilate;
  int begin_frame;
  int img_height;
  int img_width;
  Mat frame;
  double dx = 0;
  double dy = 0;
  int loopRate_ = 10;
  int if_recieve = 0;
  cv::Point P1;
  cv::Point P2;
  VideoCapture capture;
  long frameToStart;
  vector<Rect> roi;
  int hand_signal = 1;
  int ifdetect = 0;
  // MultiTracker trackers;
  vector<my_target> cars;
  vector<my_target> new_cars;

  vector<Rect2d> obj;
  vector<Ptr<Tracker>> algorithms;
  // vector<cv::Point> history_pos;
  std_msgs::Int8 if_track;
  bool ok = 0;
  Point2f GlobalCenter;
  float Radius;
  vector<kcf::KCFTracker*> tracker;
  ros::Publisher roi_pub;
  ros::Publisher if_track_pub;
  ros::Subscriber hand_sub;
  ros::Subscriber bounding_sub;
  vector<Scalar> colors;

  image_transport::Subscriber camera_subscriber;
  iarc_msgs::RoiPos roi_pos;
  vector<darknet_ros_msgs::BoundingBox> roi_recv;
  void hand_callback(const std_msgs::Int8::ConstPtr& msg);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void bounding_box_callback(const darknet_ros_msgs::BoundingBoxes& msg);
  void getRandomColors(vector<Scalar>& colors, int numColors);
  void get_roi();
  void basic_detection();
  void mainloop();
  Tracking_Melon();
  ~Tracking_Melon();
  void init();
  friend class my_target;
};

// Ptr<Tracker> createTrackerByName(string trackerType)
// {
//   Ptr<Tracker> tracker;
//   if (trackerType == trackerTypes[0])
//     tracker = TrackerBoosting::create();
//   else if (trackerType == trackerTypes[1])
//     tracker = TrackerMIL::create();
//   else if (trackerType == trackerTypes[2])
//     tracker = TrackerKCF::create();
//   else if (trackerType == trackerTypes[3])
//     tracker = TrackerTLD::create();
//   else if (trackerType == trackerTypes[4])
//     tracker = TrackerMedianFlow::create();
//   else if (trackerType == trackerTypes[5])
//     tracker = TrackerGOTURN::create();
//   else if (trackerType == trackerTypes[6])
//     tracker = TrackerMOSSE::create();
//   else if (trackerType == trackerTypes[7])
//     tracker = TrackerCSRT::create();
//   else
//   {
//     cout << "Incorrect tracker name" << endl;
//     cout << "Available trackers are: " << endl;
//     for (vector<string>::iterator it = trackerTypes.begin(); it !=
//     trackerTypes.end(); ++it)
//     {
//       std::cout << " " << *it << endl;
//     }
//   }
//   return tracker;
// }
#endif
