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
  cv::Mat image = cv::imread("/home/shubh/Documents/lane data/frames/f46.jpg", CV_LOAD_IMAGE_COLOR);
  cv::waitKey(30);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  ros::Rate loop_rate(0.2);
  while (nh.ok()) {
    ros::Time time = ros::Time::now();
    (msg->header).stamp = time;
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}