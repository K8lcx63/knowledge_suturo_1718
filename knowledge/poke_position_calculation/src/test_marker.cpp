#include <ros/ros.h>
#include <marker_publisher/marker_publisher.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");

    MarkerPublisher white_marker_publisher("white", Color::WHITE);
    MarkerPublisher black_marker_publisher("black", Color::BLACK);
    MarkerPublisher brown_marker_publisher("brown", Color::BROWN);
    MarkerPublisher red_marker_publisher("red", Color::RED);
    MarkerPublisher green_marker_publisher("green", Color::GREEN);
    MarkerPublisher blue_marker_publisher("blue", Color::BLUE);
    MarkerPublisher yellow_marker_publisher("yellow", Color::YELLOW);
    MarkerPublisher orange_marker_publisher("orange", Color::ORANGE);
    MarkerPublisher purple_marker_publisher("purple", Color::PURPLE);

    geometry_msgs::PointStamped white_marker_point;
    white_marker_point.header.stamp = ros::Time();
    white_marker_point.header.frame_id = "/map";
    white_marker_point.point.x = 0;
    white_marker_point.point.y = 0;
    white_marker_point.point.z = 0;

    geometry_msgs::PointStamped black_marker_point;
    black_marker_point.header.stamp = ros::Time();
    black_marker_point.header.frame_id = "/map";
    black_marker_point.point.x = 0.5;
    black_marker_point.point.y = 0;
    black_marker_point.point.z = 0;

    geometry_msgs::PointStamped brown_marker_point;
    brown_marker_point.header.stamp = ros::Time();
    brown_marker_point.header.frame_id = "/map";
    brown_marker_point.point.x = 1.0;
    brown_marker_point.point.y = 0;
    brown_marker_point.point.z = 0;

    geometry_msgs::PointStamped red_marker_point;
    red_marker_point.header.stamp = ros::Time();
    red_marker_point.header.frame_id = "/map";
    red_marker_point.point.x = 1.5;
    red_marker_point.point.y = 0;
    red_marker_point.point.z = 0;

    geometry_msgs::PointStamped green_marker_point;
    green_marker_point.header.stamp = ros::Time();
    green_marker_point.header.frame_id = "/map";
    green_marker_point.point.x = 2.0;
    green_marker_point.point.y = 0;
    green_marker_point.point.z = 0;

    geometry_msgs::PointStamped blue_marker_point;
    blue_marker_point.header.stamp = ros::Time();
    blue_marker_point.header.frame_id = "/map";
    blue_marker_point.point.x = 2.5;
    blue_marker_point.point.y = 0;
    blue_marker_point.point.z = 0;

    geometry_msgs::PointStamped yellow_marker_point;
    yellow_marker_point.header.stamp = ros::Time();
    yellow_marker_point.header.frame_id = "/map";
    yellow_marker_point.point.x = 3.0;
    yellow_marker_point.point.y = 0;
    yellow_marker_point.point.z = 0;

    geometry_msgs::PointStamped orange_marker_point;
    orange_marker_point.header.stamp = ros::Time();
    orange_marker_point.header.frame_id = "/map";
    orange_marker_point.point.x = 3.5;
    orange_marker_point.point.y = 0;
    orange_marker_point.point.z = 0;

    geometry_msgs::PointStamped purple_marker_point;
    purple_marker_point.header.stamp = ros::Time();
    purple_marker_point.header.frame_id = "/map";
    purple_marker_point.point.x = 4.0;
    purple_marker_point.point.y = 0;
    purple_marker_point.point.z = 0;

    ROS_INFO_STREAM("marker test is running...");
    ros::Rate rate(10);
    while(ros::ok())
    {
        white_marker_publisher.publishVisualizationMarker(white_marker_point);
        black_marker_publisher.publishVisualizationMarker(black_marker_point);
        brown_marker_publisher.publishVisualizationMarker(brown_marker_point);
        red_marker_publisher.publishVisualizationMarker(red_marker_point);
        green_marker_publisher.publishVisualizationMarker(green_marker_point);
        blue_marker_publisher.publishVisualizationMarker(blue_marker_point);
        yellow_marker_publisher.publishVisualizationMarker(yellow_marker_point);
        orange_marker_publisher.publishVisualizationMarker(orange_marker_point);
        purple_marker_publisher.publishVisualizationMarker(purple_marker_point);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}