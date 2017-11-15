#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  ros::Rate loop_rate(0.5);
  sensor_msgs::ImagePtr msg;
  cv::Mat image;
  int i = 46;
  while(nh.ok()) {
    image = cv::imread(cv::format("/home/shubh/catkin_ws/src/gridmapper/data/f%d.jpg",i), CV_LOAD_IMAGE_COLOR);
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    ros::Time time = ros::Time::now();
    (msg->header).stamp = time;
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}