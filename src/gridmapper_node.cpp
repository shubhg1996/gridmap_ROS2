#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <math.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "gridmapper/classify.h"
#include "stlastar.h"
#include <nav_msgs/Path.h>

#define RESOLUTION 0.01
#define RES2 0.001
#define step 0.005/RES2
#define ALGO 0

using namespace grid_map;
using namespace cv;
int MaxH = 480;
int MaxW = 640;
Mat cammat = (Mat_<double>(3,3) <<
700, 0, 320,
0, 700, 240,
0, 0, 1);

// Path
nav_msgs::Path path_msg;

//global findobj params
std::vector<Point3f> tvec;
std::vector<Point3f> rvec;
Mat curimg;

//tf2
tf2_ros::Buffer tfBuffer;
ros::ServiceClient client;

// Create grid map.
GridMap map_({"elevation"});

//classify
float cluster(Mat roi,int flag) {
  gridmapper::classify srv;
  cv_bridge::CvImage roi_ptr(std_msgs::Header(), "bgr8", roi);
  roi_ptr.toImageMsg(srv.request.image);
  srv.request.flag = flag;
  ROS_INFO("Calling service");
  if (client.call(srv))
    return srv.response.val;
  else
    return 0.5;
}

//project imagepoint and objectness calculation
float findObj(Position imgpos, std::vector<Point3f> rvec, std::vector<Point3f> tvec, Mat img, int flag) {
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
  return cluster(image_roi, flag);
}

float GetMap( int x, int y ,int flag)
{
  float xnew = x*RES2;
  float ynew = y*RES2;
  if(xnew<=-0.6 || ynew<=0 || xnew>=0.6 || ynew>=2.0)
    return 1;
  ROS_INFO("GetMap call: %f xnew %f ynew", xnew, ynew);
  Position position(xnew,ynew);
  float rval = map_.atPosition("elevation",position);
  // float rval = 0.5;
  if(rval<0.6 && rval>0.4){
    rval = findObj(position,rvec,tvec,curimg, flag)*rval*2.0;
    map_.atPosition("elevation",position) = rval;
  }
	return rval;
}

float GetMap2( int x, int y )
{
  float a1,a2,a3,a4,a5;
  a1 = GetMap(x+step,y,0);
  a2 = GetMap(x,y+step,0);
  a3 = GetMap(x-step,y,0);
  a4 = GetMap(x,y-step,0);
  if(a1+a2+a3+a4>2.5)
    a5 = GetMap(x,y,1);
  else if(a1+a2+a3+a4<1.5)
    a5 = GetMap(x,y,-1);
  else
    a5 = GetMap(x,y,0);
  return a5;
}

// Definitions

class MapSearchNode
{
public:
	int x;	 // the (x,y) positions of the node
	int y;	
	
	MapSearchNode() { x = y = 0; }
	MapSearchNode( int px, int py ) { x=px; y=py; }

	float GoalDistanceEstimate( MapSearchNode &nodeGoal );
	bool IsGoal( MapSearchNode &nodeGoal );
	bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
	float GetCost( MapSearchNode &successor );
	bool IsSameState( MapSearchNode &rhs );

	void PrintNodeInfo(); 


};

bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{

	// same state in a maze search is simply when (x,y) are the same
	if( (x == rhs.x) &&
		(y == rhs.y) )
	{
		return true;
	}
	else
	{
		return false;
	}

}

void MapSearchNode::PrintNodeInfo()
{
  ROS_INFO("Node position : (%d,%d)\n", x,y);
}

float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
  int dist = sqrt((x - nodeGoal.x)*(x - nodeGoal.x) + (y - nodeGoal.y)*(y - nodeGoal.y));
  if( dist < 0.02/RES2)
	{
		return 0;
	}
    return ALGO*dist+(1-ALGO)*100;
}

bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
{

	if( GoalDistanceEstimate(nodeGoal) == 0)
	{
		return true;
	}

	return false;
}

bool MapSearchNode::GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node )
{

	int parent_x = -1; 
	int parent_y = -1; 

	if( parent_node )
	{
		parent_x = parent_node->x;
		parent_y = parent_node->y;
	}

	MapSearchNode NewNode;

	// push each possible move except allowing the search to go backwards

	if( (GetMap2( x-step, y ) < 0.7) 
		&& !((parent_x == x-step) && (parent_y == y))
	  ) 
	{
		NewNode = MapSearchNode( x-step, y );
		astarsearch->AddSuccessor( NewNode );
	}	

	if( (GetMap2( x, y-step ) < 0.7) 
		&& !((parent_x == x) && (parent_y == y-step))
	  ) 
	{
		NewNode = MapSearchNode( x, y-step );
		astarsearch->AddSuccessor( NewNode );
	}	

	if( (GetMap2( x+step, y ) < 0.7)
		&& !((parent_x == x+step) && (parent_y == y))
	  ) 
	{
		NewNode = MapSearchNode( x+step, y );
		astarsearch->AddSuccessor( NewNode );
	}	

		
	if( (GetMap2( x, y+step ) < 0.7) 
		&& !((parent_x == x) && (parent_y == y+step))
		)
	{
		NewNode = MapSearchNode( x, y+step );
		astarsearch->AddSuccessor( NewNode );
	}	

	return true;
}

float MapSearchNode::GetCost( MapSearchNode &successor )
{
	return (float) GetMap2( x, y );

}


//Image callback for gridmap generation
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  ros::Time imgtime = (msg->header).stamp;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped = tfBuffer.lookupTransform("map", "camera", imgtime, ros::Duration(1.0));  
  tvec.clear();
  tvec.push_back(Point3f(transformStamped.transform.translation.x,transformStamped.transform.translation.y,transformStamped.transform.translation.z));
  tf::Quaternion q(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  rvec.clear();
  rvec.push_back(Point3f(roll,pitch,yaw));
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);
  curimg = (cv_ptr->image).clone();

  
  //Astar
  AStarSearch<MapSearchNode> astarsearch;

  // Create a start state
  MapSearchNode nodeStart;
  nodeStart.x = transformStamped.transform.translation.x/RES2;
  nodeStart.y = transformStamped.transform.translation.y/RES2 + 1; 

  // Define the goal state
  MapSearchNode nodeEnd;
  nodeEnd.x = (transformStamped.transform.translation.x-0.1)/RES2;						
  nodeEnd.y = (transformStamped.transform.translation.y+0.2)/RES2; 
  
  // Set Start and goal states
  
  astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );

  unsigned int SearchState;
  unsigned int SearchSteps = 0;

  do
  {
    SearchState = astarsearch.SearchStep();

    SearchSteps++;
  }
  while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );
  ROS_INFO("SearchSteps: %d", SearchSteps);
  MapSearchNode *node = astarsearch.GetSolutionStart();
  path_msg.poses.resize(SearchSteps);
  for(int i=0; i<100; i++){
    path_msg.poses[i].pose.position.x = node->x*RES2;
    path_msg.poses[i].pose.position.y = node->y*RES2;
    node = astarsearch.GetSolutionNext();
    if( !node )
    {
      break;
    }
  }
  astarsearch.FreeSolutionNodes();
  // astarsearch.EnsureMemoryFreed();
  
  // Position center(-0.01,0.01);
  // int out = 0;
  // Position center(transformStamped.transform.translation.x, transformStamped.transform.translation.y);
  // ROS_INFO("Got image with tf %f x %f y .",
  // transformStamped.transform.translation.x, transformStamped.transform.translation.y);

  // Position center(0.0,0.15);
  // double radius = 0.1;
  // for (grid_map::CircleIterator it(map_, center, radius); !it.isPastEnd(); ++it) {
  //   Position position;
  //   map_.getPosition(*it, position);
  //   ROS_INFO("Positions %f dx %f dy .",position.x()-(float)((int)(position.x()/RES2))*RES2, position.y()-(float)((int)(position.y()/RES2))*RES2);
  //   float out = GetMap(position.x()/RES2,position.y()/RES2);//findObj(position,rvec,tvec,curimg);
    // float rval = findObj(position,rvec,tvec,curimg);
    // map_.atPosition("elevation",position) = rval;
  // }
}

//main
int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "grid_map_simple_demo");
  ros::NodeHandle nh("~");
	ros::Publisher pub0 = nh.advertise<nav_msgs::Path>("planned_path", 1);
  ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/camera/image", 1, imageCallback);
  client = nh.serviceClient<gridmapper::classify>("/classify");

  // Path
  path_msg.header.frame_id = "map";

  //map settings
  map_.setFrameId("map");
  map_.setGeometry(Length(1.2, 2.0), RESOLUTION, Position(0.0, 1.0));
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map_.getLength().x(), map_.getLength().y(),
    map_.getSize()(0), map_.getSize()(1));
  
  //listener
  tf2_ros::TransformListener tfListener(tfBuffer);

  // Work with grid map in a loop.
  ros::Rate rate(30.0);
  map_["elevation"].setConstant(0.5);
  while (nh.ok()) {
    // Add data to grid map.
    ros::Time time = ros::Time::now();

    // Publish grid map.
    map_.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map_, message);
    publisher.publish(message);
    pub0.publish(path_msg);
    // ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());

    // Wait for next cycle.
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}

