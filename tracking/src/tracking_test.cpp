#include "tracking_test.h"
Tracking_Melon::Tracking_Melon()
{
  init();
}

Tracking_Melon::~Tracking_Melon(){}

void Tracking_Melon::init(){
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  roi_pub = nh.advertise<iarc_msgs::RoiPos>("RoiPose", 30);
  bounding_sub = nh.subscribe("/darknet_ros/bounding_boxes",1,&Tracking_Melon::bounding_box_callback, this);
  camera_subscriber =it.subscribe("/video_pub/image_detect", 1, &Tracking_Melon::imageCallback, this);
  if_track_pub = nh.advertise<std_msgs::Int8>("/tracking/if_tracking",30);
   ros::spinOnce();

}
void Tracking_Melon::bounding_box_callback(const darknet_ros_msgs::BoundingBoxes& msg){
    if(!ok){
    roi_recv = msg.bounding_boxes;
    ifdetect=0;
    
    if(roi_recv.size()){
    for(int i =0;i<roi_recv.size();i++){
      roi.push_back(cv::Rect(roi_recv[i].xmin,roi_recv[i].ymin,roi_recv[i].xmax-roi_recv[i].xmin,roi_recv[i].ymax-roi_recv[i].ymin));
    
    }
    for (auto i = 0; i < roi.size(); i++) {
        obj.push_back(roi[i]);
        algorithms.push_back(TrackerKCF::create());
     }
     ifdetect=1;
  }}
}

void Tracking_Melon::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    if(img.rows&&img.cols){
      if_recieve=1;
      frame = img.clone();
      img_width = frame.rows;
      img_height = frame.cols;
    }
    //  cv::flip(frame, frame, 0);
    // cv::imshow("Flip", frame);
    // cv::waitKey(30);  
}

void Tracking_Melon::mainloop(){
  ros::Rate loop_rate_class(loopRate_);
  const char winName[]="My Camera";
  cv::namedWindow(winName,1);
  int k=0;
  while(ros::ok()){
       ros::spinOnce();
    if_track.data = 0;
     vector<cv::Point> history_pos;
    trackers.add(algorithms, frame, obj);
    if(if_recieve==1){
    ok = trackers.update(frame);
    // cout<<"okkkk   "<<ok<<endl;
    while (ok) {
      k++;
      ros::spinOnce();
      ok = trackers.update(frame);

        // cout<<"trackers.getObjects().size()          "<<trackers.getObjects().size()<<endl;
        for (auto j = 0; j < trackers.getObjects().size(); j++) {
            rectangle(frame, trackers.getObjects()[j], Scalar(255, 0, 0), 2, 1);
             history_pos.push_back(Point(trackers.getObjects()[j].x + 
              cvRound(trackers.getObjects()[j].width/2.0),trackers.getObjects()[j].y 
              + cvRound(trackers.getObjects()[j].height/2.0)));
              cout<<trackers.getObjects().size()<<endl;
//             rect.tl();       //返回rect的左上顶点的坐标 [100, 50]
// rect.br();       //返回rect的右下顶点的坐标 [150, 150]
// rect.width();    //返回rect的宽度 50
// rect.height();   //返回rect的高度 100
            // history_pos[j].push_back(Point(trackers.getObjects()[j].x + 
            //   cvRound(trackers.getObjects()[j].width/2.0),trackers.getObjects()[j].y 
            //   + cvRound(trackers.getObjects()[j].height/2.0)));
            
            for(auto i = 0;i<history_pos.size();i++){
           circle(frame, history_pos[i], 3,Scalar(255,0,0),-1);  
          }
        
        }
        if_track.data = 1;
          imshow(winName, frame);
        cv::waitKey(10);  
 
        // if_track_pub.publish(if_track);  
    }
    // else{
    //     if_track.data = 0;
    //     // if_track_pub.publish(if_track);
 
    // }
        //   imshow(winName, frame);
        // cv::waitKey(10);  
 
    // roi_pos.dx=dx;
    // roi_pos.dy=dy;
    // roi_pos.detectornot=1;
    // roi_pub.publish(roi_pos);

   loop_rate_class.sleep();
  }
    if_track_pub.publish(if_track);
    
}
}


