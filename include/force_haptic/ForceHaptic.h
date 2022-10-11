
#pragma once

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include <boost/thread/thread.hpp>
#include "iiwa_msgs/JointPosition.h"
#include <memory>
#include "geometry_msgs/WrenchStamped.h"



#define MAX_FORCE 10.0f

class HapticForce
{
    public:
        HapticForce(ros::NodeHandle node, float loopRate, std::string forceTopic, std::string sensorForceTopic, std::string robotForceTopic, std::string robotPositionTopic);
        ~HapticForce();

        void PublishForceData();

        void ProcessForce(double x, double y, double z);
        
        void GetInputData();

        void startForceNode();

        //comentariu
        //alt comentariu
   
    private:
        ros::NodeHandle node;
        ros::Rate loopRate;
      

        ros::Publisher force_pub;
        ros::Subscriber sensor_force_sub;
        ros::Subscriber robot_force_sub;
        ros::Subscriber robot_position_sub;

        std::string sensorForceTopic;
        std::string robotForceTopic;
        std::string robotPositionTopic;
        std::string forceTopic;

        geometry_msgs::Vector3 sensorForces;
        geometry_msgs::Vector3 robotForces;
        geometry_msgs::Vector3 outputForces;

        double sensor_forces[3];
        double robot_forces[7];
        double robot_position[7];
        double output_forces[3];

        void SensorForceCallBack(const geometry_msgs::Vector3::ConstPtr &data);
        void RobotForceCallBack(const geometry_msgs::WrenchStamped::ConstPtr &data);
        void RobotPositionCallBack(const iiwa_msgs::JointPosition::ConstPtr &data);

};


