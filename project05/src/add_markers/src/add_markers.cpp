#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <nav_msgs/Odometry.h>

struct Position {float x, y;};
const Position pick_up_pos = {5, 7}; // x, y
const Position drop_off_pos = {6, -7}; // x, y

ros::Publisher marker_pub;
visualization_msgs::Marker marker;
enum MarkerState {
  Ready, Delievering, Delievered
};
MarkerState marker_state = MarkerState::Ready;

void show_marker(const Position & pos) {
  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pos.x;
  marker.pose.position.y = pos.y;
  marker_pub.publish(marker);
}


void hide_marker() {
  marker.header.stamp = ros::Time::now();
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
}


void odom_sub_cb(const nav_msgs::Odometry::ConstPtr& msg) {
  float x = msg->pose.pose.position.x;
  float y = msg->pose.pose.position.y;
  float tolerance = 0.4;

  if (marker_state == MarkerState::Ready && 
      (fabs(x-pick_up_pos.x) + fabs(y-pick_up_pos.y)) < tolerance) {
    hide_marker();
    ROS_INFO("Virtual object is picked up and shipped");
    ROS_INFO("x = %f, y = %f", x, y);
    marker_state = MarkerState::Delievering;
  } else if (marker_state == Delievering &&
             (fabs(x-drop_off_pos.x) + fabs(y-drop_off_pos.y)) < tolerance) {
    show_marker(drop_off_pos);
    ROS_INFO("Virtual object is dropped off");
    ROS_INFO("x = %f, y = %f", x, y);
    marker_state = MarkerState::Delievered;
  }
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "add_markers");
  ros::NodeHandle private_n("~");
	ros::NodeHandle n;
	ros::Rate r(1);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  while (!ros::ok()) {
    ROS_INFO("Waiting for ROS to setup");
		r.sleep();
	}

  while (marker_pub.getNumSubscribers() < 1) {
    ROS_WARN("Please create a subscriber to the marker");
    sleep(1);
  }

  uint32_t shape = visualization_msgs::Marker::CUBE;

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  marker.ns = "add_markers";
  marker.id = 0;
  marker.type = shape;
  // marker.action = visualization_msgs::Marker::ADD;

  // marker.pose.position.x = 0;
  // marker.pose.position.y = 0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Initial state.
  show_marker(pick_up_pos);
  ROS_INFO("Virtual object is waiting to be picked up");
    
  std::string mode;
	private_n.getParam("mode", mode);

  if (mode != "service") {
    // Default is Time Mode. 
    ROS_INFO("Time Mode: each state will last 5 seconds.");
    
    ros::Duration(5).sleep();
    hide_marker();
    ROS_INFO("Virtual object is picked up and shipped");
    
    ros::Duration(5).sleep();
    show_marker(drop_off_pos);
    ROS_INFO("Virtual object is dropped off");
  } else {
    // Enter Service Mode.
    ROS_INFO("Service Mode: listen to the robot's behavior.");

    ros::Subscriber odom_sub = n.subscribe("odom", 1000, odom_sub_cb);
    ros::spin();
  }

  while (ros::ok()) {
    sleep(1);
  }
}