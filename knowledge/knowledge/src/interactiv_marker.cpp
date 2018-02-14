#include <ros/ros.h>
#include <interactiv_marker_publisher/interactiv_marker_publisher.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "interactiv_marker");
   
  InteractivMarkerPublisher pub;
  pub.addInteractivMarker("test", 0.58, 0.375, Color::RED);

  ros::spin();
   
  return 0;
}