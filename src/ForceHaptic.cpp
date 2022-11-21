#include <force_haptic/ForceHaptic.h>

 // #define DEBUGG ///> debugging tool

HapticForce::HapticForce(ros::NodeHandle node, float loopRate, std::string forceTopic, 
                         std::string sensorForceTopic, std::string robotForceTopic, std::string robotPositionTopic):
                         node(node), loopRate(loopRate), forceTopic(forceTopic), sensorForceTopic(sensorForceTopic),
                         robotForceTopic(robotForceTopic), robotPositionTopic(robotPositionTopic){
                            
                            this->force_pub = this->node.advertise<geometry_msgs::Vector3>(this->forceTopic.c_str(),1);
                            
                            this->sensor_force_sub = this->node.subscribe<geometry_msgs::Vector3>(this->sensorForceTopic.c_str(), 1, &HapticForce::SensorForceCallBack, this);
                            this->robot_force_sub = this->node.subscribe<geometry_msgs::WrenchStamped>(this->robotForceTopic.c_str(), 1, &HapticForce::RobotForceCallBack, this);
                            this->robot_position_sub = this->node.subscribe<iiwa_msgs::JointPosition>(this->robotPositionTopic.c_str(), 1, &HapticForce::RobotPositionCallBack, this);
}

HapticForce::~HapticForce(){
}

void HapticForce::PublishForceData(){
   
   ros::AsyncSpinner spinner(2);
   spinner.start();

   while(ros::ok()){

      geometry_msgs::Vector3 currentOutputForce;

      #ifdef DEBUGG
         //this is the initial DEBUGG logic
         currentOutputForce.x = (sensorForces.x + robotForces.x)/2;
         currentOutputForce.y = (sensorForces.y + robotForces.y)/2;
         currentOutputForce.z = (sensorForces.z + robotForces.z)/2;

      #endif

      ProcessForce(currentOutputForce.x, currentOutputForce.y, currentOutputForce.z);

      

      this->force_pub.publish(outputForces);
      #ifdef DEBUGG
         std::cout<< sensorForces.x <<" ";
         std::cout<< sensorForces.y <<" ";
         std::cout<< sensorForces.z <<"\n";
      #endif
      this->loopRate.sleep();
   }

   spinner.stop();


}

void HapticForce::ProcessForce(double x, double y, double z){


   /**====================================== //
    * ********Force Limitations************* //
    * ====================================== //
   */                                        //
   if (x < -MAX_FORCE) x = -MAX_FORCE;       //
   if (y < -MAX_FORCE) y = -MAX_FORCE;       //
   if (z < -MAX_FORCE) z = -MAX_FORCE;       //
   if (x > MAX_FORCE) x = MAX_FORCE;         //
   if (y > MAX_FORCE) y = MAX_FORCE;         //
   if (z > MAX_FORCE) z = MAX_FORCE;         //
                                             //   
   //========================================//

   /**=======================================//
    * ***********Sine values**************** //
    * =======================================//
   */                                        //
   double s1 = sin(this->robot_position[0]); //
   double s2 = sin(this->robot_position[1]); //
   double s3 = sin(this->robot_position[2]); //
   double s4 = sin(this->robot_position[3]); //
   double s5 = sin(this->robot_position[4]); //
   double s6 = sin(this->robot_position[5]); // 
   double s7 = sin(this->robot_position[6]); //
                                             //
   //========================================//

   /**=======================================//
    * **********Cosine Values*************** //
    * =======================================//
   */                                        //      
   double c1 = cos(this->robot_position[0]); //
   double c2 = cos(this->robot_position[1]); //
   double c3 = cos(this->robot_position[2]); //
   double c4 = cos(this->robot_position[3]); //
   double c5 = cos(this->robot_position[4]); //
   double c6 = cos(this->robot_position[5]); //
   double c7 = cos(this->robot_position[6]); //
                                             //
   //========================================//                                          

   /*===============================================================
    **********Wierd math Formuli Generated in MATLAB****************
    *===============================================================
    * 
    * the reference frame transformation takes place here in order to relocate TCP forces into base frame
   */
   double x1 = z*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) 
               + s5*(c3*s1 + c1*c2*s3))) - x*(c7*(s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) 
               + s5*(c3*s1 + c1*c2*s3))) - s7*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1* + c1*c2*s3))) 
               - y*(c7*(s5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*(c3*s1* + c1*c2*s3)) + s7*(s6*(s4*(s1*s3 - c1*c2*c3) 
               + c1*c4*s2) + c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + s5*(c3*s1 + c1*c2*s3))));

   double y1 = x*(c7*(s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) 
               - s7*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*(c1*c3 - c2*s1*s3))) - z*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) 
               - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + s5*(c1*c3 - c2*s1*s3))) + y*(c7*(s5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) 
               - c5*(c1*c3 - c2*s1*s3)) + s7*(s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) 
               + s5*(c1*c3 - c2*s1*s3))));

   double z1 = z*(s6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) + c6*(c2*c4 + c3*s2*s4)) - x*(s7*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) 
               - c7*(c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - s6*(c2*c4 + c3*s2*s4))) + y*(c7*(s5*(c2*s4 - c3*c4*s2) - c5*s2*s3) 
               + s7*(c6*(c5*(c2*s4 - c3*c4*s2) + s2*s3*s5) - s6*(c2*c4 + c3*s2*s4)));

    outputForces.x = x1;   //for vector attribution
    outputForces.y = y1;   //
    outputForces.z = z1;   //
}

void HapticForce::GetInputData(){
   //std::cout<<"zis ist ein tird test";
   //this is basically unused and could be naturally BANISHED INTO NONEXISTANCE@horvathd2
}

void HapticForce::startForceNode(){
  
   PublishForceData();
   
   //this function is redundant, might be a good idea to be removed
}


void HapticForce::SensorForceCallBack(const geometry_msgs::Vector3::ConstPtr &data){
   sensorForces.x = data->x;
   sensorForces.y = data->y;
   sensorForces.z = data->z;
}

void HapticForce::RobotForceCallBack(const geometry_msgs::WrenchStamped::ConstPtr &data){
   geometry_msgs::Wrench wrench;
   wrench = data->wrench;
   robotForces = wrench.force;
}
void HapticForce::RobotPositionCallBack(const iiwa_msgs::JointPosition::ConstPtr &data){
   robot_position[0] = data->position.a1;
   robot_position[1] = data->position.a2;
   robot_position[2] = data->position.a3;
   robot_position[3] = data->position.a4;
   robot_position[4] = data->position.a5;
   robot_position[5] = data->position.a6;
   robot_position[6] = data->position.a7;
}