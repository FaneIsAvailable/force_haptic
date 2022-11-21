#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <memory>
#include <force_haptic/ForceHaptic.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "force_haptic_node");

    ros::NodeHandle node;

    HapticForce force(node, 4000,"/chai3d/force", "/robotiq_ft_wrench", "force/robot_force", "/iiwa/state/JointPosition");

    force.startForceNode();

    ros::shutdown();

}