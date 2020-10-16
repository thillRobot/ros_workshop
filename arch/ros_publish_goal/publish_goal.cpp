#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <sstream>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_goal");
    ros::NodeHandle n;
    ros::Publisher ttu_publisher =
    n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
    ros::Rate loop_rate(10);

    geometry_msgs::PoseStamped msg;
    msg.header.stamp=ros::Time::now();
    msg.header.frame_id="map";

    int count = 0;
    while ((ros::ok())&&(count<5))  
    {               
        msg.pose.position.x = 3.0;
        msg.pose.position.y = 2.0;
        msg.pose.position.z = 0;
        msg.pose.orientation.w = 1.0;

        ttu_publisher.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
}