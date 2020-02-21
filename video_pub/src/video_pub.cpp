#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int8.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include "iarc_msgs/Start_Cure.h"
#include <string>
using namespace std;
using namespace cv;
int if_track = 0;
int if_start_track = 0;

void if_track_callback(const std_msgs::Int8& msg) { if_track = msg.data; }
// void if_start_track_callback(const iarc_msgs::Start_Cure &msg){
//     if_start_track = msg.start_cure;
// }
int main(int argc, char** argv) {
  ros::init(argc, argv, "video_pub");
  ros::NodeHandle nh;
  ros::Subscriber if_track_sub;
  ros::Subscriber if_start;
  string video_name;
  // cout << "if_read"
  //      << nh.param<std::string>(
  //             "video_name", video_name,
  //             "/media/gjx/GJX/video_pic/VID_20190918_223654.mp4")
  //      << endl;
  ros::param::get("~video_name", video_name);
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/video_pub/image_detect", 1);
  image_transport::Publisher pub2 = it.advertise("/video_pub/image_track", 1);
  if_track_sub = nh.subscribe("/tracking/if_tracking", 10, if_track_callback);
  // if_start = nh.subscribe("/iarc_missions/if_start_track", 10,
  // if_start_track_callback);

  // if (!nh.getParam("video_name", video_name))
  //   video_name = "/home/gjx/视频/VID_20191222_154610.mp4";
  Mat myframe = imread("/tmp/mozilla_gjx0/image00038.jpg");
  int choose_flag = 1;
  VideoCapture capture(video_name);

  int img_width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
  int img_height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);

  if (!capture.isOpened()) {
    cout << "can not read video" << endl;
  }

  long totalFrameNumber = capture.get(CV_CAP_PROP_FRAME_COUNT);
  cout << "整个视频共" << totalFrameNumber << "帧" << endl;
  int frameToStop = 4500;
  //获取帧率
  double rate = capture.get(CV_CAP_PROP_FPS);
  cout << "帧率为:" << rate << endl;

  int frameToStart = 1;
  capture.set(CV_CAP_PROP_POS_FRAMES, frameToStart);
  cout << "从第" << frameToStart << "帧开始读" << endl;
  long currentFrame = frameToStart;
  // waitKey( 10000 );
  Mat frame;
  cv::Mat image;
  ros::Rate loop_rate(5);
  while (nh.ok()) {
    if (choose_flag) {
      ros::spinOnce();
      if (currentFrame == totalFrameNumber - 5) {
        capture.set(CV_CAP_PROP_POS_FRAMES, frameToStart);
        currentFrame = 2;
      }
      capture >> frame;
      // cout << "正在读取第" << currentFrame << "帧" << endl;
      waitKey(10);
      image = frame.clone();
      resize(image, image, Size(img_width * 0.7, img_height * 0.7), 0, 0,
             INTER_LINEAR);
      imshow("Extractedframe", image);
      currentFrame++;
    } else {
      frame = myframe.clone();
      imshow("Extractedframe", frame);
      waitKey(40);
    }

    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    pub2.publish(msg);
    // if(!if_track){
    pub.publish(msg);
    // }
    // ros::spinOnce();
    // loop_rate.sleep();
  }
}