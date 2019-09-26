#include "tracking_test.h"
using namespace std;


int main(int argc,char** argv){
  ros::init(argc, argv, "tracking_node");
  Tracking_Melon tracking_test;
  tracking_test.mainloop();
  return 0;
}