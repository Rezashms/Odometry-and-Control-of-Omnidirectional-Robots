#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2/LinearMath/Matrix3x3.h"
class checkParams {

public:
    checkParams() {
        /*
        message_filters::Subscriber<sensor_msgs::JointState> wheelSub(n, "/wheel_states", 1);
        message_filters::Subscriber<geometry_msgs::PoseStamped> poseSub(n, "/robot/pose", 1);
        message_filters::TimeSynchronizer<geometry_msgs::PoseStamped, sensor_msgs::JointState> sync (poseSub, wheelSub, 10);
        sync.registerCallback(boost::bind(&checkParams::bagCallback, this, _1, _2));
        */
        this->wheelSub=this->n.subscribe("/wheel_states", 1000, &checkParams::wheelSubCallback,this);
        this->poseSub=this->n.subscribe("/robot/pose", 1000, &checkParams::poseSubCallback,this);

        this->paramCount = 0;
        this->wheelCount = 0;
        this->poseCount = 0;
        n.getParam("/gearRatio", this->gearRatio);
        n.getParam("/wheelRadius", this->wheelRad);
        n.getParam("/halfLenght", this->halfLength);
        n.getParam("/halfWidth", this->halfWidth);
        n.getParam("/tickRes", this->tickRes);
    }

    void mainLoop(){
        ROS_INFO("Param node started");
        ros::spin();
    }

    void wheelSubCallback(const sensor_msgs::JointState::ConstPtr& msg) {

        if(this->wheelCount == 0) {
            this -> ticfl = msg->position[0];
            this -> ticfr = msg->position[1];
            this -> ticrl = msg->position[2];
            this -> ticrr = msg->position[3];
        }
        else if(this->wheelCount % 100 == 0) {
            ticfl0 = ticfl;
            ticfr0 = ticfr;
            ticrl0 = ticrl;
            ticrr0 = ticrr;

            this -> ticfl = msg->position[0];
            this -> ticfr = msg->position[1];
            this -> ticrl = msg->position[2];
            this -> ticrr = msg->position[3];
            
            paramCalibration();
        }
        
        ++wheelCount;
    }

    void poseSubCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

        if(this->poseCount == 0) {
            ts = msg->header.stamp;
            x = msg -> pose.position.x;
            y = msg -> pose.position.y;
            quat_msg = msg -> pose.orientation;
            tf2::fromMsg(quat_msg, q);
            tf2::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
            theta = yaw;
        }
        else if(this->poseCount % 100 == 0) {
            ts0 = ts;
            x0 = x;
            y0 = y;
            theta0 = theta;

            ts = msg->header.stamp;
            x = msg -> pose.position.x;
            y = msg -> pose.position.y;
            quat_msg = msg -> pose.orientation;
            tf2::fromMsg(quat_msg, q);
            tf2::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
            theta = yaw;

            paramCalibration();
        }
        
        ++poseCount;
    }

    void paramCalibration() {

        if(wheelCount >= 100 && poseCount >= 100) {

            deltats = ts - ts0;
            wz = (theta - theta0)/deltats.toSec();
            vx = (sin(theta0)*(y-y0)+cos(theta0)*(x-x0))/deltats.toSec();
            vy = (cos(theta0)*(y-y0)-sin(theta0)*(x-x0))/deltats.toSec();
            ROS_INFO("vx %f | vy %f | wz %f", vx, vy, wz);

            if (paramCount%2 == 0) {
                tempValue = (wz*deltats.toSec()*gearRatio*tickRes*(halfLength + halfWidth)*4)/(2*M_PI*(-(ticfl-ticfl0)+(ticfr-ticfr0)-(ticrl-ticrl0)+(ticrr-ticrr0)));
                ROS_INFO("tempValue R: %f", tempValue);
            }
            if (paramCount%2 == 1) {
                tempValue = ((-(ticfl-ticfl0)+(ticfr-ticfr0)-(ticrl-ticrl0)+(ticrr-ticrr0))*2*M_PI*wheelRad)/(deltats.toSec()*gearRatio*wz*(halfLength + halfWidth)*4);
                ROS_INFO("tempValue N: %f", tempValue);
            }
            //ROS_INFO("R: %f | N: %f, tempValue: %f", wheelRad, tickRes, tempValue);

            ++paramCount;
        }
    }


private:
    ros::NodeHandle n;
    ros::Subscriber wheelSub;
    ros::Subscriber poseSub;
    double x, y, theta;
    double x0, y0, theta0;
    double roll, pitch, yaw;
    double wz, vx, vy;
    ros::Time ts, ts0;
    ros::Duration deltats;
    tf2::Quaternion q;
    geometry_msgs::Quaternion quat_msg;
    int paramCount, wheelCount, poseCount;
    double wheelRad, halfLength, halfWidth, tickRes, gearRatio, tempValue;
    double ticfl, ticfr, ticrr, ticrl;
    double ticfl0, ticfr0, ticrr0, ticrl0;
};

int main (int argc, char **argv) {
    ros::init(argc, argv, "checkParams");
    checkParams checkPar;
    checkPar.mainLoop();
    return 0;
}