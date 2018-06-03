/*
 * Author: Juan Neyra
 * Description: Detects a ball of a selected color
 *
*/
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>

#include "opencv2/core/utility.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"

using namespace std;
using namespace cv;

static void help(){
  cout << "\nChoose one of twelves colors in the window, then the\n"
       << "program will track the ball of the selected color\n"
       << "The only parameter is the number of the camera port.\n"
       << "Press ESC key to exit the program.\n";
}

int main(int argc, char** argv){

  VideoCapture cap;
  Mat frame, imgHSV, imgFinal;
  int color = 0;

  const int numColors = 12;
  const char* windowName = "Ball_tracker";
  /*const int red = 0, orange = 1, yellow = 2, lime = 3, green = 4,
    turquoise = 5, cyan = 6, skyBlue = 7, blue = 8, purple = 9,
    magenta = 10, redMagenta = 11;*/
  const int factor = 21;
  const int range = 12;
  const int satMin = 110, valMin = 30;

  if(argc != 2) { help(); return -1; }
  int camPort = (*argv[1]) - 48;
  cap.open(camPort);
  if( !cap.isOpened() ){
      help();
      cout << "Camera port cannot be open" << endl;
      return -1;
  }

  // ROS
  ros::init(argc, argv, "tracker_node");
  ros::NodeHandle nh;
  ros::Publisher pub_center =
              nh.advertise<geometry_msgs::Point>("/tracker_center",1000);
  //ros::Publisher pub_size =
  //            nh.advertise<geometry_msgs::Vector3>("tracker_size",1000);
  geometry_msgs::Point msg_pub_center;
  msg_pub_center.z = 0;
  //geometry_msgs::Vector3 msg_pub_size;
  //msg_pub_size.z = 0;

  namedWindow(windowName, CV_WINDOW_NORMAL);
  createTrackbar("Color", windowName, &color, numColors-1);

  Mat planes[3];
  Mat hue1, hue2;
  Rect searchWindow = Rect(0, 0, 20, 20);
  Point center;
  Mat element =  getStructuringElement(MORPH_ELLIPSE,
                                       Size(3, 3),
                                       Point(2, 2));
  Mat element2 =  getStructuringElement(MORPH_ELLIPSE,
                                      Size(5, 5),
                                      Point(2, 2));
  Mat img;
  Mat map_x, map_y;


  while(ros::ok()){
    cap.read(frame);
    if( frame.empty() ) {
      cout << "No frame was detected." << endl;
      break;
    }
    // Blurring the image
    GaussianBlur( frame, img, Size( 7, 7 ), 0, 0 );

    cvtColor(img, imgHSV, COLOR_BGR2HSV);
    split(imgHSV, planes);
    normalize(planes[0], planes[0], 0, 255, NORM_MINMAX);

    if(color==0){
        threshold(planes[0], hue1, 255-range, 255, 0);
        threshold(planes[0], hue2, range, 255, 1);
        planes[0] = hue1 | hue2;
    }
    else{
      threshold(planes[0], hue1, color*factor-range, 255, 0);
      threshold(planes[0], hue2, color*factor+range, 255, 1);
      planes[0] = hue1 & hue2;
    }

    threshold(planes[1], planes[1], satMin, 255, 0);
    threshold(planes[2], planes[2], valMin, 255, 0);

    imgFinal = planes[0] & planes[1] & planes[2];
    erode(imgFinal, imgFinal, element2);
    //erode(imgFinal, imgFinal, element);
    dilate(imgFinal, imgFinal, element);
    dilate(imgFinal, imgFinal, element);

    if( searchWindow.area() <= 10 ) { searchWindow = Rect(0,0,20,20); }
    RotatedRect objectRect = CamShift(imgFinal, searchWindow,
              TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 10, 1));

    circle(frame, objectRect.center, 3, Scalar(0,255,0), -1, 8, 0);
    ellipse(frame, objectRect, Scalar(0,0,255), 2, CV_AA);

    imshow("imgFinal", imgFinal);
    imshow("Ball_tracker", frame);

    msg_pub_center.x = objectRect.center.x;
    msg_pub_center.y = objectRect.center.y;
//    msg_pub_size.x = frame.size().width;
//    msg_pub_size.y = frame.size().height;
    pub_center.publish(msg_pub_center);
//    pub_size.publish(msg_pub_size);

    char c = (char)waitKey(10);
    if( c == 27 ) { break; }
  }

  return 0;
}
