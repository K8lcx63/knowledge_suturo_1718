#include <ros/ros.h>
#include <marker_publisher/marker_publisher.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker");
   
  geometry_msgs::PointStamped center_sauce;
  center_sauce.header.frame_id = "iai_kitchen/sink_area_surface";
  center_sauce.point.x = 0.0; 
  center_sauce.point.y = -0.845;
  center_sauce.point.z = 0.0;

  geometry_msgs::PointStamped center_snacks;
  center_snacks.header.frame_id = "iai_kitchen/sink_area_surface";
  center_snacks.point.x = 0.0;
  center_snacks.point.y = -0.47;
  center_snacks.point.z = 0.0;

  geometry_msgs::PointStamped center_breakfast_food;
  center_breakfast_food.header.frame_id = "iai_kitchen/sink_area_surface";
  center_breakfast_food.point.x = 0.0;
  center_breakfast_food.point.y = -0.095;
  center_breakfast_food.point.z = 0.0;

  geometry_msgs::PointStamped center_food_vessel;
  center_food_vessel.header.frame_id = "iai_kitchen/sink_area_surface";
  center_food_vessel.point.x = 0.0;
  center_food_vessel.point.y = 0.28;
  center_food_vessel.point.z = 0.0;

  geometry_msgs::PointStamped test_point;
  test_point.header.frame_id = "/map";
  test_point.point.x = 1.2102882188707471;
  test_point.point.y = -0.9779632384870455;
  test_point.point.z = 3.994443646016858;

  geometry_msgs::PointStamped test_point_2;
  test_point_2.header.frame_id = "/head_mount_kinect_ir_optical_frame";
  test_point_2.point.x = 1.0;
  test_point_2.point.y = -2.5;
  test_point_2.point.z = 1.4;


  MarkerPublisher sauce_pub("sauce", Color::RED, 0.58, 0.375);
  MarkerPublisher snacks_pub("snacks", Color::BLUE, 0.58, 0.375);
  MarkerPublisher breakfast_food_pub("breakfastFood", Color::YELLOW, 0.58, 0.375);
  MarkerPublisher food_vessel_pub("foodVessels", Color::WHITE, 0.58, 0.375);
  MarkerPublisher test_pub("test", Color::BLACK);
  MarkerPublisher test_pub_2("test_2", Color::ORANGE);


  ros::Rate rate(10);
    while(ros::ok())
    {
        sauce_pub.publishVisualizationMarker(center_sauce);
        snacks_pub.publishVisualizationMarker(center_snacks);
        breakfast_food_pub.publishVisualizationMarker(center_breakfast_food);
        food_vessel_pub.publishVisualizationMarker(center_food_vessel);
        test_pub.publishVisualizationMarker(test_point);
        test_pub_2.publishVisualizationMarker(test_point_2);

        ros::spinOnce();
        rate.sleep();
    }
   
  return 0;
}