#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <memory>
#include <force_haptic/ForceHaptic.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "force_haptic_node");

    ros::NodeHandle node;
    //node.setParam("/gripper_mass", 2.3);
    HapticForce force(node, 4000,"/chai3d/force", "/robotiq_ft_wrench", "force/robot_force", "/iiwa/state/JointPosition");
    node.param<double>("/gripper_mass", force.gripperMass, 0);

    force.startForceNode();

    ros::shutdown();

}