#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "project_1/ResetPose.h"
#include <tf2/LinearMath/Quaternion.h>
#include <dynamic_reconfigure/server.h>
#include <project_1/parametersConfig.h>

class ComputeOdometry {

public:

    ComputeOdometry() {
        
        this->velInput=this->n.subscribe("/cmd_vel", 1000, &ComputeOdometry::callOdometryMethod, this);
        this->odomPub=this->n.advertise<nav_msgs::Odometry>("/odom", 1000);
        this-> resetService = this->n.advertiseService("resetPose", &ComputeOdometry::resetCallback, this);
                
        this->x0 = 0.0;
        this->y0 = 0.0;
        this->theta0 = 0.0;
        this->ts0.sec = 0;
        this->ts0.nsec = 0;
        this->set_method = 0;       // Euler integration method is set to default


        // dynamic reconfigure setup
        this->f = boost::bind(&ComputeOdometry::parametersCallback, this, _1, _2);
        this->dynServer.setCallback(f);
    }

    void mainLoop(){
        ROS_INFO("Odometry node started\n");

        ros::spin();
    }

    double computeTimeStamp(geometry_msgs::TwistStamped::ConstPtr &msg) {
        // function that returns a timestamp variable of type double, from the
        // variables sec and nsec of type uint32_t contained in the header of the message

        return msg->header.stamp.sec + msg->header.stamp.nsec * pow(10, -9);
    }

    void publishOdometry(const geometry_msgs::TwistStamped::ConstPtr &msg) {
        // function that creates and publishes a nav_msgs::Odometry message on topic /odom

        nav_msgs::Odometry odomMsg;
        tf2::Quaternion q;
        q.setRPY(0, 0, this->theta0);

        odomMsg.child_frame_id = "base_link";
        odomMsg.header.frame_id = "odom";
        odomMsg.header.stamp = msg -> header.stamp;
        odomMsg.pose.pose.position.x = this -> x0;
        odomMsg.pose.pose.position.y = this -> y0;
        odomMsg.pose.pose.position.z = 0.0;
        odomMsg.pose.pose.orientation.x = q.x();
        odomMsg.pose.pose.orientation.y = q.y();
        odomMsg.pose.pose.orientation.z = q.z();
        odomMsg.pose.pose.orientation.w = q.w();
        this -> odomPub.publish(odomMsg);
    }

    void callOdometryMethod(const geometry_msgs::TwistStamped::ConstPtr &msg) {
        // This function is executed everytime a message is received and calls the
        // correct integration method function, according to the set_method variable

        if(set_method == 0) {
            eulerOdometry(msg);
        }
        else {
            rungeKuttaOdometry(msg);
        }
    }

    void eulerOdometry(const geometry_msgs::TwistStamped::ConstPtr &msg) {
        // 1st order Euler integration, rotation is performed after the translation

        if(ts0.nsec == 0) {
            // When the first message is received, the new timestamp is saved but no
            // odometry is performed since we only have the first value of v and omega
            ts0 = msg->header.stamp;
        }

        else {
            ts = msg->header.stamp;
            deltats = ts - ts0;
            theta = theta0 + msg->twist.angular.z * deltats.toSec();
            x = x0 + deltats.toSec() * (msg->twist.linear.x * cos (theta0) - msg->twist.linear.y * sin (theta0));
            y = y0 + deltats.toSec() * (msg->twist.linear.x * sin (theta0) + msg->twist.linear.y * cos (theta0));
            theta0 = theta;
            x0 = x;
            y0 = y;
            ts0 = ts;

            // creating and publishing the odometry message
            publishOdometry(msg);
        }
    }

    void rungeKuttaOdometry(const geometry_msgs::TwistStamped::ConstPtr &msg) {
        // 2nd order Runge Kutta integration, theta changes during the translation

        if(ts0.nsec == 0.0) {
            // When the first message is received, the new timestamp is saved but no
            // odometry is performed since we only have the first value of v and omega
            ts0 = msg->header.stamp;
        }

        else {
            // starting from the second message received, the odometry is computed
            ts = msg->header.stamp;
            deltats = ts - ts0;
            theta_avg = theta0 + msg->twist.angular.z * deltats.toSec() / 2;
            theta = theta0 + msg->twist.angular.z * deltats.toSec();
            x = x0 + deltats.toSec() * (msg->twist.linear.x*cos(theta_avg) - msg->twist.linear.y*sin(theta_avg));
            y = y0 + deltats.toSec() * (msg->twist.linear.x*sin(theta_avg) + msg->twist.linear.y*cos(theta_avg));

            // updating past values with current values
            theta0 = theta;
            x0 = x;
            y0 = y;
            ts0 = ts;

            // creating and publishing the odometry message
            publishOdometry(msg);
        }
    }

    bool resetCallback(project_1::ResetPose::Request  &req, project_1::ResetPose::Response &res) {
        // reset the robot pose to the desired values

        res.x_old = this->x0;
        res.y_old = this->y0;
        res.theta_old = this->theta0;

        this->x0 = req.x_new;
        this->y0 = req.y_new;
        this->theta0 = req.theta_new;

        this->ts0.sec = 0;
        this->ts0.nsec = 0;

        //publishOdometry(msg);
        nav_msgs::Odometry odomMsg;
        tf2::Quaternion q;
        q.setRPY(0, 0, this->theta0);

        odomMsg.child_frame_id = "base_link";
        odomMsg.header.frame_id = "odom";
        odomMsg.header.stamp = this->ts;
        odomMsg.pose.pose.position.x = this -> x0;
        odomMsg.pose.pose.position.y = this -> y0;
        odomMsg.pose.pose.position.z = 0.0;
        odomMsg.pose.pose.orientation.x = q.x();
        odomMsg.pose.pose.orientation.y = q.y();
        odomMsg.pose.pose.orientation.z = q.z();
        odomMsg.pose.pose.orientation.w = q.w();
        this -> odomPub.publish(odomMsg);


        ROS_INFO("\nOld pose: (%lf,%lf,%lf)\nNew pose: (%lf,%lf,%lf)", 
            res.x_old, res.y_old, res.theta_old, req.x_new, req.y_new, req.theta_new);

        return true;
    }

    void parametersCallback(project_1::parametersConfig &config, uint32_t level) {
        // integration method is changed
        
        if(level == 0) {
            if(config.set_method == 0) {  
                this->set_method = 0;       // Euler
                ROS_INFO("Dynamic reconfigure: integration method set to Euler");
            }
            else {
                this->set_method = 1;       // Runge-Kutta
                ROS_INFO("Dynamic reconfigure: integration method set to Runge-Kutta");
            }
        }
    }

private:

    ros::NodeHandle n;
	ros::Subscriber velInput;
    ros::Publisher odomPub;
    ros::ServiceServer resetService;

    dynamic_reconfigure::Server<project_1::parametersConfig> dynServer;
    dynamic_reconfigure::Server<project_1::parametersConfig>::CallbackType f;

    double x, y, theta;
    double x0, y0, theta0;
    double theta_avg;
    ros::Time ts,ts0;
    ros::Duration deltats;
    int set_method;         // integration method: Euler = 0 , RK = 1
};

int main (int argc, char **argv) {

    ros::init(argc, argv, "ComputeOdometry");

    ComputeOdometry compOdo;
    compOdo.mainLoop();

    return 0;
}