#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class tfBroadcast
{
public:
    tfBroadcast(){
        this -> odomSub = n.subscribe("/odom", 1000, &tfBroadcast::transformCallback, this);
    }

    void transformCallback(const nav_msgs::Odometry::ConstPtr &msg){

        //Transformation from odometry frame to robot frame
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = msg->pose.pose.position.x;
        transformStamped.transform.translation.y = msg->pose.pose.position.y;
        transformStamped.transform.translation.z = msg->pose.pose.position.z;
        transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
        transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
        transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
        transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;

        toBaseLBr.sendTransform(transformStamped);

        //Transformation from world frame (fixed frame) to odometry frame
        static_transformStamped.header.stamp = ros::Time::now();
        static_transformStamped.header.frame_id = "world";
        static_transformStamped.child_frame_id = "odom";
        static_transformStamped.transform.translation.x = 0;
        static_transformStamped.transform.translation.y = 0;
        static_transformStamped.transform.translation.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        static_transformStamped.transform.rotation.x = q.x();
        static_transformStamped.transform.rotation.y = q.y();
        static_transformStamped.transform.rotation.z = q.z();
        static_transformStamped.transform.rotation.w = q.w();
    
        static_broadcaster.sendTransform(static_transformStamped);


    }
private:
    ros::NodeHandle n;
    ros::Subscriber odomSub;

    tf2_ros::TransformBroadcaster toBaseLBr;
    geometry_msgs::TransformStamped transformStamped;

    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "tfBroadcast");
  tfBroadcast tfBroadcaster;
  ros::spin();
  return 0;
}