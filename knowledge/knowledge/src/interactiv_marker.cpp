#include <ros/ros.h>
#include <interactiv_marker_publisher/interactiv_marker_publisher.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interactiv_marker");
  ros::NodeHandle nh("~");
   
  InteractivMarkerPublisher pub(nh);
  pub.make6DofMarker();

  ros::spin();
   
  return 0;
}