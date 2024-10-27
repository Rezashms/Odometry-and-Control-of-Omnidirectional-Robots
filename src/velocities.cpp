#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"

class ComputeVelocity {
public:
	ComputeVelocity(){
		this->sensorInput=this->n.subscribe("/wheel_states", 1000, &ComputeVelocity::sensorCallback,this);
		this->velocitiesPub=this->n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1000);

        n.getParam("/gearRatio", this->gearRatio);
        n.getParam("/wheelRadius", this->wheelRadius);
        n.getParam("/halfLenght", this->halfLength);
        n.getParam("/halfWidth", this->halfWidth);
	}
    void mainLoop(){
        ROS_INFO("Robot velocity node started\n");
        ros::spin();
        
    }
    void sensorCallback(const sensor_msgs::JointState::ConstPtr& msg){
        //Acquisition of wheels' angular velocities (front-left, front-right, rear-left, rear-right)
        wfl = msg->velocity[0]/(60*gearRatio);
        ROS_INFO("Rot fl: %f", wfl);
        wfr = msg->velocity[1]/(60*gearRatio);
        ROS_INFO("Rot fr: %f", wfr);
        wrl = msg->velocity[2]/(60*gearRatio);
        ROS_INFO("Rot rl: %f", wrl);
        wrr = msg->velocity[3]/(60*gearRatio);
        ROS_INFO("Rot rr: %f", wrr);

        //Computation of robot frame velocities
        vx = (wfl+wfr+wrl+wrr)*wheelRadius/4;
        ROS_INFO("Vel x: %f", vx);
        vy = (-wfl+wfr+wrl-wrr)*wheelRadius/4;
        ROS_INFO("Vel y: %f", vy);
        wz = (-wfl+wfr-wrl+wrr)*wheelRadius/(4*(halfWidth+halfLength));
        ROS_INFO("W z: %f", wz);

        //Pubblication of robot frame velocities on topic /wheel_states
        geometry_msgs::TwistStamped velMsg;
        velMsg.header.stamp.sec = msg->header.stamp.sec;
        velMsg.header.stamp.nsec = msg -> header.stamp.nsec;
        velMsg.twist.linear.x = this -> vx;
        velMsg.twist.linear.y = this -> vy;
        velMsg.twist.angular.z = this -> wz;
        this -> velocitiesPub.publish(velMsg);

    }
private:
	ros::NodeHandle n;
	ros::Subscriber sensorInput;
	ros::Publisher velocitiesPub;
    int gearRatio;
    double wheelRadius;
    double halfLength;
    double halfWidth;
    double wfl, wfr, wrl, wrr;
    double vx, vy, wz;
};

int main (int argc, char **argv) {
	ros::init(argc, argv, "ComputeVelocities");
    ComputeVelocity compVel;
    compVel.mainLoop();
}