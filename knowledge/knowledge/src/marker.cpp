#include <ros/ros.h>
#include <marker_publisher/marker_publisher.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker");
   
  geometry_msgs::PointStamped center_sauce;
  center_sauce.header.frame_id = "/map";
  center_sauce.point.x = 1.51594; 
  center_sauce.point.y = 1.13063;
  center_sauce.point.z = 0.85;

  geometry_msgs::PointStamped center_snacks;
  center_snacks.header.frame_id = "/map";
  center_snacks.point.x = 1.51594;
  center_snacks.point.y = 0.75563;
  center_snacks.point.z = 0.85;

  geometry_msgs::PointStamped center_breakfast_food;
  center_breakfast_food.header.frame_id = "/map";
  center_breakfast_food.point.x = 1.51594;
  center_breakfast_food.point.y = 0.38063;
  center_breakfast_food.point.z = 0.85;

  geometry_msgs::PointStamped center_food_vessel;
  center_food_vessel.header.frame_id = "/map";
  center_food_vessel.point.x = 1.51594;
  center_food_vessel.point.y = 0.00563;
  center_food_vessel.point.z = 0.85;

  geometry_msgs::PointStamped test_point;
  test_point.header.frame_id = "/map";
  test_point.point.x = 1.47094;
  test_point.point.y = 0.00563;
  test_point.point.z = 0.89875;


  MarkerPublisher sauce_pub("sauce", Color::RED, 0.58, 0.375);
  MarkerPublisher snacks_pub("snacks", Color::BLUE, 0.58, 0.375);
  MarkerPublisher breakfast_food_pub("breakfastFood", Color::YELLOW, 0.58, 0.375);
  MarkerPublisher food_vessel_pub("foodVessels", Color::WHITE, 0.58, 0.375);
  MarkerPublisher test_pub("ja_milch", Color::RED);


  ros::Rate rate(10);
    while(ros::ok())
    {
        sauce_pub.publishVisualizationMarker(center_sauce);
        snacks_pub.publishVisualizationMarker(center_snacks);
        breakfast_food_pub.publishVisualizationMarker(center_breakfast_food);
        food_vessel_pub.publishVisualizationMarker(center_food_vessel);
        test_pub.publishVisualizationMarker(test_point);

        ros::spinOnce();
        rate.sleep();
    }
   
  return 0;
}