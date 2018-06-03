#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Point.h>
#include <iostream>
using namespace std;

class NodeClass
{
public:
  NodeClass()
  {
    m_pub = m_nh.advertise<std_msgs::Int8>("/servo_angle",1000);
    m_sub = m_nh.subscribe("/tracker_center", 1000,
                &NodeClass::trackerCenter, this);
  }

  void trackerCenter(const geometry_msgs::Point &msg)
  {
    std_msgs::Int8 servoAngle;
    float center_x, error;
    float i;
    center_x = msg.x;
    //ros::Rate ros_rate(1);
    if(m_count == 25){
      m_count = 0;
      cout<< "center_x: "<<center_x<<endl;
      error = m_setPoint - center_x;
      dist_ultrasonido = msg.asdf;
      m_angle = arctan

      cout << "m_angle: " << m_angle << endl<< endl;
      if(m_angle < -90){ m_angle = -90; }
      else if(m_angle > 90){ m_angle = 90; }
      servoAngle.data = m_angle;
      m_pub.publish(servoAngle);
    }
    m_count++;
  //ros_rate.sleep();
  }

private:
  ros::NodeHandle m_nh;
  ros::Publisher m_pub;
  ros::Subscriber m_sub;
  const int m_width = 640, height = 480;  // size of the frame of the camera
  // Control parameters
  const int m_setPoint = 320;
  const float m_ki = 0.03;
  const float m_kp = 0.06;
  float m_integral = 0;
  int m_angle = 0;
  int m_count = 25;
};


int main(int argc, char** argv){
  // ROS
  ros::init(argc, argv, "logic_tracker_node");
  NodeClass NCObject;

  ros::spin();

  return 0;
}
