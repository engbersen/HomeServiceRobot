// ROS Libraries
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt8.h>

//global variable keeping track of state.
uint8_t state = 0;

void atTargetCallback(const std_msgs::UInt8::ConstPtr& msg){
  ROS_INFO("Callback");
  ROS_INFO("Message: %d", msg->data);
  state = msg->data;
}

visualization_msgs::Marker createMarker(double x, double y){
  ROS_INFO("Create Marker");
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "add_markers";
  marker.id = 0;

  // Set the marker type.  
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.20;
  marker.scale.y = 0.20;
  marker.scale.z = 0.20;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  return marker;
}

void publishMarker(visualization_msgs::Marker marker, ros::NodeHandle n, ros::Publisher pub){
  //arguments are the marker, the NodeHandle and the publisher of the marker.
  // Publish the marker
  while (pub.getNumSubscribers() < 1)
    {
      /*if (!ros::ok())
      {
	    return 0;
      }*/
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    pub.publish(marker);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Subscriber target_sub = n.subscribe("/at_target", 1, atTargetCallback);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Rate loop_rate(10);
  visualization_msgs::Marker marker1;
  visualization_msgs::Marker marker2;

  while (ros::ok()){

    //determine based on callback the case.
    switch (state){
      case 0:
        //start of node
        ROS_INFO("Marker 1 created.");
        marker1 = createMarker(-1.0, 3.0);
        publishMarker(marker1, n, marker_pub);
        state = 1;
        break;
      case 1:
        //waiting for callback
        break;
      case 2:
        //robot arrived at pickup target. Delete marker.
        ROS_INFO("Marker 1 deleted.");
        marker1.action = visualization_msgs::Marker::DELETE;
        publishMarker(marker1, n, marker_pub);
        state = 3;
        break;
      case 3:
        //waiting for callback
        break;
      case 4:
        //robot arrived at drop-off target. Create marker.
        ROS_INFO("Marker 2 created.");
        marker2 = createMarker(2.0, -1.0);
        publishMarker(marker2, n, marker_pub);
        ROS_INFO("Done.");
        state = 5;
        break;
      case 5:
        sleep(1);
        break;
    }

    //ensure messages are published.
    ros::spinOnce();
    loop_rate.sleep();
  }
  //ros::spin();

  return 0;
}

