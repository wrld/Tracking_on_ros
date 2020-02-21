#include "tracking_test.h"
#define sim_method 0
Tracking_Melon::Tracking_Melon() { init(); }

Tracking_Melon::~Tracking_Melon() {}
//初始化
void Tracking_Melon::init() {
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  roi_pub = nh.advertise<iarc_msgs::RoiPos>("RoiPose", 30);
  bounding_sub = nh.subscribe("/darknet_ros/bounding_boxes", 1,
                              &Tracking_Melon::bounding_box_callback, this);
  camera_subscriber = it.subscribe("/video_pub/image_track", 1,
                                   &Tracking_Melon::imageCallback, this);
  if_track_pub = nh.advertise<std_msgs::Int8>("/tracking/if_tracking", 30);

  getRandomColors(colors, 2);
  ros::spinOnce();
}

void Tracking_Melon::getRandomColors(vector<Scalar>& colors, int numColors) {
  RNG rng(5);
  // for(int i=0; i < numColors; i++)
  // colors.push_back(Scalar(rng.uniform(0,255), rng.uniform(0, 255),
  // rng.uniform(0, 255)));
  colors.push_back(Scalar(0, 0, 255));
  colors.push_back(Scalar(0, 255, 255));
}

//接收darknet的bounding box信息
void Tracking_Melon::bounding_box_callback(
    const darknet_ros_msgs::BoundingBoxes& msg) {
  // if(!ok){

  roi_recv = msg.bounding_boxes;
  ifdetect = 0;

  if (roi_recv.size()) {
    for (int i = 0; i < roi_recv.size(); i++) {
      roi.push_back(cv::Rect(roi_recv[i].xmin, roi_recv[i].ymin,
                             roi_recv[i].xmax - roi_recv[i].xmin,
                             roi_recv[i].ymax - roi_recv[i].ymin));
    }
    for (auto i = 0; i < roi.size(); i++) {
      obj.push_back(roi[i]);
      algorithms.push_back(TrackerKCF::create());
    }
    ifdetect = 1;
  }
}
// }
//接收图片topic
void Tracking_Melon::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
  if (img.rows && img.cols) {
    if_recieve = 1;
    frame = img.clone();
    img_width = frame.rows;
    img_height = frame.cols;
  }
}

double my_target::match_sim(vector<my_target> old) {
#ifdef method
  // ssim comparision
  Scalar SSIM1 = getMSSIM(this->pic, old[0].pic);
  double ssim1 = (SSIM1.val[2] + SSIM1.val[1] + SSIM1.val[0]) / 3 * 100;
  Scalar SSIM2 = getMSSIM(this->pic, old[1].pic);
  double ssim2 = (SSIM2.val[2] + SSIM2.val[1] + SSIM2.val[0]) / 3 * 100;
  (ssim1 > ssim2) ? md_car = 0 : md_car = 1;
  cout << "sim1    " << ssim1 << "sim2    " << ssim2 << endl;
  cout << "result    " << md_car << endl;
#else
  double mindist = 1000000, md_car, simi = 0, ms_car;
  double x1 = this->bbox.x + this->bbox.width / 2;
  double y1 = this->bbox.y + this->bbox.height / 2;
  // euler distance
  for (int i = 0; i < old.size(); i++) {
    double x2 = old[i].bbox.x + old[i].bbox.width / 2;
    double y2 = old[i].bbox.y + old[i].bbox.height / 2;
    double dist = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    // cout<<"dist"<<dist;
    if (dist < mindist) {
      mindist = dist;
      md_car = i;
    }
  }
#endif
  return md_car;
}
double my_target::getMSSIM(Mat inputimage1, Mat inputimage2) {
  double C1 = 6.5025, C2 = 58.5225;
  cv::Mat image_ref = inputimage1.clone();
  cv::Mat image_obj = inputimage2.clone();
  int width = image_ref.cols;
  int height = image_ref.rows;
  int width2 = image_obj.cols;
  int height2 = image_obj.rows;
  double mean_x = 0;
  double mean_y = 0;
  double sigma_x = 0;
  double sigma_y = 0;
  double sigma_xy = 0;
  for (int v = 0; v < height; v++) {
    for (int u = 0; u < width; u++) {
      mean_x += image_ref.at<uchar>(v, u);
      mean_y += image_obj.at<uchar>(v, u);
    }
  }
  mean_x = mean_x / width / height;
  mean_y = mean_y / width / height;
  for (int v = 0; v < height; v++) {
    for (int u = 0; u < width; u++) {
      sigma_x += (image_ref.at<uchar>(v, u) - mean_x) *
                 (image_ref.at<uchar>(v, u) - mean_x);
      sigma_y += (image_obj.at<uchar>(v, u) - mean_y) *
                 (image_obj.at<uchar>(v, u) - mean_y);
      sigma_xy += abs((image_ref.at<uchar>(v, u) - mean_x) *
                      (image_obj.at<uchar>(v, u) - mean_y));
    }
  }
  sigma_x = sigma_x / (width * height - 1);
  sigma_y = sigma_y / (width * height - 1);
  sigma_xy = sigma_xy / (width * height - 1);
  double fenzi = (2 * mean_x * mean_y + C1) * (2 * sigma_xy + C2);
  double fenmu =
      (mean_x * mean_x + mean_y * mean_y + C1) * (sigma_x + sigma_y + C2);
  double ssim = fenzi / fenmu;
  return ssim;
}

bool isInside(Rect rect1, Rect rect2) { return (rect1 == (rect1 & rect2)); }

void Tracking_Melon::mainloop() {
  ros::Rate loop_rate_class(loopRate_);
  const char winName[] = "My Camera";
  cv::namedWindow(winName, 1);
  int k = 0;
  my_target tar1;
  my_target tar2;
  int colors_flag[2];
  colors_flag[0] = 0;
  colors_flag[1] = 1;
  cars.push_back(tar1);
  cars.push_back(tar2);
  new_cars.push_back(tar1);
  new_cars.push_back(tar2);
  int first = 0;

  while (ros::ok()) {
    ros::spinOnce();
    if_track.data = 0;
    vector<vector<cv::Point>> history_pos;
    vector<cv::Point> vec;
    vector<cv::Point> vec1;
    history_pos.push_back(vec);
    history_pos.push_back(vec1);
    int k = 0;
    //添加KCF算法到跟踪器中
    MultiTracker trackers;
    trackers.add(algorithms, frame, obj);

    if (ifdetect == 1 && obj.size()) {
      ok = trackers.update(frame);
      int flag = 0;
      // cout<<ok<<endl;

      while (ok) {
        k++;

        if (trackers.getObjects().size() != roi_recv.size()) {
          if_track.data = 0;
          break;
        }
        if (k == 1 && first) {
          for (auto j = 0; j < trackers.getObjects().size(); j++) {
            new_cars[j].bbox = trackers.getObjects()[j];
            //行 Row 列 Column
            if (isInside(trackers.getObjects()[j],
                         Rect(0, 0, frame.cols, frame.rows)))
              new_cars[j].pic = frame(trackers.getObjects()[j]).clone();
            double match = new_cars[j].match_sim(cars);
            new_cars[j].my_color = cars[match].my_color;
            if (match != j) {
              int temp;
              temp = colors_flag[0];
              colors_flag[0] = colors_flag[1];
              colors_flag[1] = temp;
              flag = 1;
              new_cars[j].match = match;
              break;
            }
          }
        }
        first = 1;

        // cout<<"trackers
        // "<<trackers.getObjects().size()<<"roi"<<roi_recv.size()<<endl;
        for (auto j = 0; j < trackers.getObjects().size(); j++) {
          //画出跟踪框

          rectangle(frame, trackers.getObjects()[j], colors[colors_flag[j]], 2,
                    1);
          history_pos[j].push_back(
              Point(trackers.getObjects()[j].x +
                        cvRound(trackers.getObjects()[j].width / 2.0),
                    trackers.getObjects()[j].y +
                        cvRound(trackers.getObjects()[j].height / 2.0)));

          cars[j].bbox = trackers.getObjects()[j];
          cars[j].my_color = colors[j];
          cars[j].my_history = history_pos[j];
          if (isInside(trackers.getObjects()[j],
                       Rect(0, 0, frame.cols, frame.rows)))
            cars[j].pic = frame(Rect(trackers.getObjects()[j].tl(),
                                     trackers.getObjects()[j].br()))
                              .clone();

          stringstream ss;
          string str;
          // if(flag){ss <<"mycar "<< abs(1-j);
          // cout<<"change my flag!!!!!!"<<endl;

          // }else{
          ss << "mycar " << colors_flag[j];
          // }
          str = ss.str();
          cv::rectangle(frame,
                        cv::Rect(cv::Point(trackers.getObjects()[j].x,
                                           trackers.getObjects()[j].y - 20),
                                 cv::Size(80, 20)),
                        colors[colors_flag[j]], CV_FILLED);
          cv::putText(
              frame, str,
              cv::Point(trackers.getObjects()[j].x, trackers.getObjects()[j].y),
              cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2.1);
          ///画出历史轨迹
          for (auto i = 0; i < history_pos[j].size(); i++) {
            circle(frame, history_pos[j][i], 3, colors[colors_flag[j]], -1);
          }
        }
        if_track.data = 1;
        imshow(winName, frame);
        cv::waitKey(10);
        // ros::spinOnce();
        ros::spinOnce();
        //更新跟踪
        ok = trackers.update(frame);
        if (trackers.getObjects().size() != roi_recv.size()) {
          if_track.data = 0;
          ok = 0;
        }
        // if(!ok){
        //    for (auto j = 0; j < trackers.getObjects().size(); j++) {
        //      cars[j].bbox=trackers.getObjects()[j];
        //      cars[j].my_color=colors[j];
        //      cars[j].my_history=history_pos[j];
        //      cars[j].pic=frame(trackers.getObjects()[j]).clone();
        //    }
        // cout<<"old_ok"<<endl;
        // }
      }
      roi.clear();
      algorithms.clear();
      obj.clear();
      // loop_rate_class.sleep();
    }
    if_track_pub.publish(if_track);
  }
}
