/**
 * @author @FaneIsAvailable
 * @class Haptic Force
 * @date 12.08.2022
 * @details solution for Real time Force feedback of an Omega 7 haptic device
 *          forces from kuka iiwa are used as base reference, forces from robotiq ft300 are used as additional information
 * 
*/



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
        /**
         * @brief Construct a new Haptic Force object
         * 
         * @param node ROS node
         * @param loopRate rate of the ROS node 
         * @param forceTopic output force topic name
         * @param sensorForceTopic input sensor force topic name
         * @param robotForceTopic input robot force topic name
         * @param robotPositionTopic input robot position topic name
         */
        HapticForce(ros::NodeHandle node, float loopRate, std::string forceTopic, std::string sensorForceTopic, std::string robotForceTopic, std::string robotPositionTopic);
        ~HapticForce();

        /**
         * @brief Publish all data on the ros NODE
         * 
         */
        void PublishForceData(){

        };

        /**
         * @brief force limitation in device limits
         *        force vector transformation from TCP reference frame to robot base frame
         *        tranformation equations generated in MATLAB
         * 
         * @param x x component of force vector
         * @param y y component of force vector
         * @param z z component of force vector
         */
        void ProcessForce(double x, double y, double z); 
        
        void GetInputData();// i might have forgotten to implement  this


        /**
         * @brief start the ROS node
         * @remark this also might be irrelevant
         */
        void startForceNode();
   
    private:
        ros::NodeHandle node;
        ros::Rate loopRate;
      
        /**================================
         * ********ROS node members********
         * ================================ 
        */
        ros::Publisher force_pub;           //publishes force vector to /chai3d/force
        ros::Subscriber sensor_force_sub;   //subscribes to the torque sensors main topic (@horvathd2 pune te rog numele topicului)
        ros::Subscriber robot_force_sub;    //subscribes to /iiwa_stack/state/StampedWrench
        ros::Subscriber robot_position_sub; //subscribes to /iiwa_stack/state/JointPosition

        /**================================
         * ********ROS Topic Names*********
         * ================================
        */
        std::string sensorForceTopic;
        std::string robotForceTopic;
        std::string robotPositionTopic;
        std::string forceTopic;

        /**================================
         * ******ROS published Data********
         * ================================
        */
        geometry_msgs::Vector3 sensorForces;    //input forces received from the torque sensor
        geometry_msgs::Vector3 robotForces;     //input force vector computed by kuka lbr
        geometry_msgs::Vector3 outputForces;    //resultant force vector

        /**================================
         * *****Might delete later?********
         * ================================
        */
        double sensor_forces[3];
        double robot_forces[7];
        double robot_position[7];
        double output_forces[3];

        /**================================
         * ****ROS subscriber Callback*****
         * ================================
        */
       
        void SensorForceCallBack(const geometry_msgs::Vector3::ConstPtr &data);

        void RobotForceCallBack(const geometry_msgs::WrenchStamped::ConstPtr &data);  

        void RobotPositionCallBack(const iiwa_msgs::JointPosition::ConstPtr &data);

};


