#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

using namespace grid_map;

//tf2
tf2_ros::Buffer tfBuffer;

// Create grid map.
GridMap map({"elevation"});

//Image callback for gridmacp generation
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  ros::Time imgtime = (msg->header).stamp;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped = tfBuffer.lookupTransform("camera", "map", imgtime, ros::Duration(1.0));
  Position center(transformStamped.transform.translation.x, transformStamped.transform.translation.y);
  // Position center(0.0,0.0);
  double radius = 0.4;
  for (grid_map::CircleIterator it(map, center, radius); !it.isPastEnd(); ++it) {
    Position position;
    map.getPosition(*it, position);
    map.at("elevation", *it) = 1.0;
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
  image_transport::Subscriber sub = it.subscribe("/image", 1, imageCallback);

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
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

    // Wait for next cycle.
    rate.sleep();
  }

  return 0;
}

