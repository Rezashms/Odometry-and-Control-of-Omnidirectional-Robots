#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "project_1/WheelsVel.h"

class WheelsVelocities {

public:
	WheelsVelocities(){
		this -> velInput=this->n.subscribe("/cmd_vel", 1000, &WheelsVelocities::velCallback, this);
		this -> wheelPub=this->n.advertise<project_1::WheelsVel>("/wheels_rpm", 1000);

        n.getParam("/gearRatio", this->gearRatio);
        n.getParam("/wheelRadius", this->wheelRadius);
        n.getParam("/halfLenght", this->halfLength);
        n.getParam("/halfWidth", this->halfWidth);
	}
    void mainLoop(){
        ros::Rate loop_rate(10);
        ROS_INFO("Wheels' angular velocity node started\n");
        ros::spin();
        
    }
    void velCallback(const geometry_msgs::TwistStamped::ConstPtr &msg){
        //Save robot velocities received from topic /cmd_vel
        vx = msg -> twist.linear.x;
        ROS_INFO("Vel x: %f", vx);
        vy = msg -> twist.linear.y;
        ROS_INFO("Vel y: %f", vy);
        wz = msg -> twist.angular.z;
        ROS_INFO("W z: %f", wz);

        //Computed wheels' angular speeds from robot velocities (front-left, front-right, rear-left, rear-right)
        wfl = 1/wheelRadius * (vx - vy - (halfLength + halfWidth) * wz);
        ROS_INFO("Rot fl: %f", wfl);
        wfr = 1/wheelRadius * (vx + vy + (halfLength + halfWidth) * wz);
        ROS_INFO("Rot fr: %f", wfr);
        wrl = 1/wheelRadius * (vx + vy - (halfLength + halfWidth) * wz);
        ROS_INFO("Rot rr: %f", wrl);
        wrr = 1/wheelRadius * (vx - vy + (halfLength + halfWidth) * wz);
        ROS_INFO("Rot rl: %f", wrr);

        //Create custom wheel velocities message and publish it on topic /wheels_rpm
        project_1::WheelsVel wheelsMsg;
        wheelsMsg.rpm_fl = wfl;
        wheelsMsg.rpm_fr = wfr;
        wheelsMsg.rpm_rr = wrr;
        wheelsMsg.rpm_rl = wrl;
        this -> wheelPub.publish(wheelsMsg);

    }
private:
	ros::NodeHandle n;
	ros::Subscriber velInput;
	ros::Publisher wheelPub;
    int gearRatio;
    double wheelRadius;
    double halfLength;
    double halfWidth;
    double wfl, wfr, wrl, wrr;
    double vx, vy, wz;
};

int main (int argc, char **argv) {
	ros::init(argc, argv, "WheelsVelocities");
    WheelsVelocities wheelsVel;
    wheelsVel.mainLoop();
}