#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "gridmapper/classify.h"

using namespace grid_map;
using namespace cv;
int MaxH = 480;
int MaxW = 640;
Mat cammat = (Mat_<double>(3,3) <<
700, 0, 320,
0, 700, 240,
0, 0, 1);


//tf2
tf2_ros::Buffer tfBuffer;
ros::ServiceClient client;

// Create grid map.
GridMap map({"elevation"});

//classify
int cluster(Mat roi) {
  gridmapper::classify srv;
  cv_bridge::CvImage roi_ptr(std_msgs::Header(), "bgr8", roi);
  roi_ptr.toImageMsg(srv.request.image);
  ROS_INFO("Calling service");
  if (client.call(srv))
    return srv.response.val;
  else
    return 0.5;
}

//project imagepoint and objectness calculation
int findObj(Position imgpos, std::vector<Point3f> rvec, std::vector<Point3f> tvec, Mat img) {
  std::vector<Point3f> imgpt;
  std::vector<Point2f> imgout;
  imgpt.push_back(Point3f(imgpos.x(),imgpos.y(),0.0));
  //project points on image and extract ROI
  projectPoints(imgpt,rvec,tvec,cammat,noArray(),imgout);
  ROS_INFO("Projected pt: %f x %f y", imgout[0].x, imgout[0].y);
  if(imgout[0].x>(MaxW-6) || imgout[0].x<(6) || imgout[0].y>(MaxH-6) || imgout[0].y<(6))
    return 0.5;
  Rect region_of_interest = Rect((int)imgout[0].x-4, (int)(MaxH-imgout[0].y)-4, 10, 10);
  Mat image_roi = img(region_of_interest);
  //get cluster class
  // imshow("frame",image_roi);
  // waitKey(30);
  // return 0;
  return cluster(image_roi);
}

//Image callback for gridmacp generation
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  ros::Time imgtime = (msg->header).stamp;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped = tfBuffer.lookupTransform("map", "camera", imgtime, ros::Duration(1.0));
  std::vector<Point3f> tvec;
  tvec.push_back(Point3f(transformStamped.transform.translation.x,transformStamped.transform.translation.y,transformStamped.transform.translation.z));
  std::vector<Point3f> rvec;
  tf::Quaternion q(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  rvec.push_back(Point3f(roll,pitch,yaw));
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
  Position center(0.0,0.15);
  // int out = 0;
  // Position center(transformStamped.transform.translation.x, transformStamped.transform.translation.y);
  // ROS_INFO("Got image with tf %f x %f y .",
  // transformStamped.transform.translation.x, transformStamped.transform.translation.y);

  // // Position center(0.0,0.4);
  double radius = 0.1;
  for (grid_map::CircleIterator it(map, center, radius); !it.isPastEnd(); ++it) {
    Position position;
    map.getPosition(*it, position);
    map.at("elevation", *it) = findObj(position,rvec,tvec,cv_ptr->image);
  }
}

//main
int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "grid_map_simple_demo");
  ros::NodeHandle nh("~");
  ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/camera/image", 1, imageCallback);
  client = nh.serviceClient<gridmapper::classify>("/classify");

  //map settings
  map.setFrameId("map");
  map.setGeometry(Length(1.2, 2.0), 0.03, Position(0.0, 1.0));
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));
  
  //listener
  tf2_ros::TransformListener tfListener(tfBuffer);

  // Work with grid map in a loop.
  ros::Rate rate(30.0);
  map["elevation"].setConstant(0.5);
  while (nh.ok()) {
    // Add data to grid map.
    ros::Time time = ros::Time::now();

    // Publish grid map.
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    publisher.publish(message);
    // ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

    // Wait for next cycle.
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}

