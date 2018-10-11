/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */
/* Modified by: Fu-Jen Chu, Ruinian Xu*/

#include <ros/ros.h>
#include <ros/duration.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <dynamixel_msgs/JointState.h>

#include <time.h>
#include <chrono>
#include <unistd.h>
#include <math.h>
#include <unistd.h>
#include <cstdlib> 
#include <iostream>
#include <cmath>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions> 
#include <tf/tf.h>

// include headers for moveit group
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

double time_pick = 0.0;
double time_place = 0.0;

std::vector<double> current_joint_values;
bool mutex = 0;
const double pi = std::acos(-1);

void joint_state_handler(const sensor_msgs::JointState::ConstPtr& msg);
void traj_end_mutex(const std_msgs::Bool::ConstPtr& msg);
void mutex_traj();
const std::vector<std::string> listNamedPoses(moveit::planning_interface::MoveGroupInterface &group);
void gotoNamedTarget(moveit::planning_interface::MoveGroupInterface &group, std::string target, bool constraint_on);
void addObject2Scene(moveit::planning_interface::MoveGroupInterface &group, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, ros::Publisher &collision_object_publisher);
void getConstraint(moveit::planning_interface::MoveGroupInterface &group, std::string target);
void adjust(geometry_msgs::Pose &target_pose);

int main(int argc, char **argv)
{

    /*****************************************************************
    *                         Arm initialization                     *
    *****************************************************************/
    // ros initialization
    ros::init(argc, argv, "arm_kinematics");
    ros::NodeHandle node_handle;  
    ros::Publisher pub_8 = node_handle.advertise<std_msgs::Float64>("/finalarm_position_controller_8/command", 1, true);

    ros::Subscriber sub_multi_joint_states = node_handle.subscribe<sensor_msgs::JointState>("/joint_states", 1, joint_state_handler);
    ros::Subscriber sub_endtime = node_handle.subscribe<std_msgs::Bool>("/finalarm_joint_trajectory_action_controller/mutex", 5, traj_end_mutex);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // kinematic_state & kinematic_model loading & planner
    moveit::planning_interface::MoveGroupInterface group_l_arm("left_arm");
    moveit::planning_interface::MoveGroupInterface group_r_arm("right_arm");
    group_l_arm.setPlannerId("BKPIECEkConfigDefault");//ForageRRTkConfigDefault//LBKPIECEkConfigDefault//RRTstarkConfigDefault//BKPIECEkConfigDefault//RRTstarkConfigDefault
    group_r_arm.setPlannerId("BKPIECEkConfigDefault");
    // relationship is as follow
    // robot_state::RobotStatePtr kinematic_state:       group.getCurrentState() 
    // robot_model::RobotModelPtr kinematic_model:       group.getCurrentState()->getRobotModel()
    // robot_state::JointModelGroup* joint_model_group:  group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName())
    
    // We will use the planning_scene_interface class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 

    /*****************************************************************
    *                         ROS info output                        *
    *****************************************************************/
    // get the joint names to verify if loaded successfully 
    const std::vector<std::string> &l_joint_names = group_l_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_l_arm.getName())->getJointModelNames();
    const std::vector<std::string> &r_joint_names = group_r_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_r_arm.getName())->getJointModelNames();
    //std::cout << "Loading model..  joints in arm model:" << std::endl; 
    //for (size_t i = 0; i < 8 ; i++) std::cout << joint_names[i] << std::endl;

    const std::vector<std::string> &l_link_names = group_l_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_l_arm.getName())->getLinkModelNames();
    const std::vector<std::string> &r_link_names = group_r_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_r_arm.getName())->getLinkModelNames();
    //for (size_t i = 0; i < 8 ; i++) std::cout << link_names[i] << std::endl;

    ROS_INFO("Model frame: %s", group_l_arm.getCurrentState()->getRobotModel()->getModelFrame().c_str());
    ROS_INFO("Reference Planning frame: %s", group_l_arm.getPlanningFrame().c_str());
    ROS_INFO("EndEffectorLink simulator: %s", group_l_arm.getEndEffectorLink().c_str());
    ROS_INFO("Model frame: %s", group_r_arm.getCurrentState()->getRobotModel()->getModelFrame().c_str());
    ROS_INFO("Reference Planning frame: %s", group_r_arm.getPlanningFrame().c_str());
    ROS_INFO("EndEffectorLink simulator: %s", group_r_arm.getEndEffectorLink().c_str());

    /*****************************************************************
    *                       Visualization setup                      *
    *****************************************************************/
    // for visualization
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    /*****************************************************************
    *                     Adding objects to world                    *
    *****************************************************************/
    /* First put an object into the scene*/
    /* Advertise the collision object message publisher*/
    ros::Publisher collision_object_publisher = node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
    while(collision_object_publisher.getNumSubscribers() < 1) {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }
    // addObject2Scene(group_arm, planning_scene_interface ,collision_object_publisher);
  
    /*****************************************************************
    *                        List stored poses                       *
    *****************************************************************/
    // list the pre-defined pose in SRDF file
    const std::vector<std::string> namedTargets_l = listNamedPoses(group_l_arm);
    const std::vector<std::string> namedTargets_r = listNamedPoses(group_r_arm);

    /*****************************************************************
    *                      Specify initial pose                      *
    *****************************************************************/
    // namedTargets stores the names of pre-defined poses(joint values)
    // select the name (ex. "home"), gotoNamedTarget function will find plan to the specific pose

    std::string target = ""; 
    int targetNum = 0;
    std::cout<<"select target pose above: "; std::cin >> targetNum;
    if(targetNum == 0) target = "home";
    else target = "home";

    
    gotoNamedTarget(group_l_arm, target + "_l", 0);
    gotoNamedTarget(group_r_arm, target + "_r", 0);

    /*
    std::string command = "";
    std::string pos_x = "";
    std::string pos_y = "";
    std::cout<< "Handy is waiting for command! (you might need to run the server)"<<std::endl;
    */

    int flag_arm = 0;
    while(flag_arm != 1 && flag_arm != 2){
        std::cout<<"Which arm you want to manipulate? 1. Right 2. Left"<<std::endl;
        std::cin>>flag_arm;
        switch(flag_arm){
            case 1: std::cout<<"We are about to manipuate the right arm."<<std::endl;
            case 2: std::cout<<"We are about to manipuate the left arm."<<std::endl;
            default: std::cout<<"Wrong input, please choose 1 or 2."<<std::endl;
        }
    }
    
    int flag = 1;
    while (flag) {
    
        /*****************************************************************
        *                         Attempt to pick up                     *
        *****************************************************************/
        // specify the target_pose
        // the robot will attempt to:
        // 1. go above the object
        // 2. open gripper
        // 3. go down
        // 4. close gripper
        // 5.  lift gripper a little bit
        // 6. go back to home position
        int flag_manipulation_done = 0;
        while (!flag_manipulation_done){
            geometry_msgs::Pose pose;

            double x,y,z,angle;

            std::cout<<"Type the x position:"<<std::endl;std::cin>>x;
            std::cout<<"Type the y position:"<<std::endl;std::cin>>y;
            std::cout<<"Type the z position:"<<std::endl;std::cin>>z;
            std::cout<<"Type the rotation angle of wrist:"<<std::endl;std::cin>>angle;

            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = z;
            tf::Quaternion qat = tf::createQuaternionFromRPY(0, -M_PI, M_PI + angle);
            qat.normalize();
            pose.orientation.x = qat.x();//0.577;//0.49; // two-sided gribber
            pose.orientation.y = qat.y();//0.577;//0.49; // two-sided gribber
            pose.orientation.z = qat.z();//0.577;//0.49;
            pose.orientation.w = qat.w(); 
            
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            moveit::planning_interface::MoveItErrorCode success;

            if(flag_arm == 1){
                group_r_arm.setPoseTarget(pose);
                // plan
                success = group_r_arm.plan(my_plan);
                int count = 1;
                while (!success){
                    success = group_r_arm.plan(my_plan);
                    if (count > 10){
                        std::cout<<"Plan for input pose failed ten times, please input another pose"<<std::endl;
                        break;
                    }
                }
                // visualization
                //ROS_INFO("Visualizing plan 1 (pose goal) %s",success == moveit_msgs::MoveItErrorCodes::SUCCESS?"":"FAILED");    
                // execute
                if(success){
                    group_r_arm.execute(my_plan);
                    mutex_traj();
                    flag_manipulation_done = 1;
                }
                
            }
            else{
                group_l_arm.setPoseTarget(pose);
                // plan
                success = group_l_arm.plan(my_plan);
                int count = 1;
                while (!success){
                    success = group_l_arm.plan(my_plan);
                    if (count > 10){
                        std::cout<<"Plan for input pose failed ten times, please input another pose"<<std::endl;
                        break;
                    }
                }
                // visualization
                //ROS_INFO("Visualizing plan 1 (pose goal) %s",success == moveit_msgs::MoveItErrorCodes::SUCCESS?"":"FAILED");    
                // execute
                if(success){
                    group_l_arm.execute(my_plan);
                    mutex_traj();
                    flag_manipulation_done = 1;
                }
            }
        }

        std::cout<<"Would you like to continue picking items? 0.No 1.Yes"<<std::endl;
        std::cin >> flag;

        flag_arm = 0;
        while(flag_arm != 1 && flag_arm != 2){
            std::cout<<"Which arm you want to manipulate? 1. Right 2. Left"<<std::endl;
            std::cin>>flag_arm;
            switch(flag_arm){
                case 1: std::cout<<"We are about to manipuate the right arm."<<std::endl;
                case 2: std::cout<<"We are about to manipuate the left arm."<<std::endl;
                default: std::cout<<"Wrong input, please choose 1 or 2."<<std::endl;
            }
        }
    }

    ros::spin();
    return 0;
}

void joint_state_handler(const sensor_msgs::JointState::ConstPtr& msg)
{
    current_joint_values.clear();
    for(std::size_t i = 0; i < msg->position.size(); ++i) {
        current_joint_values.push_back(msg->position[i]);
    }
}

void traj_end_mutex(const std_msgs::Bool::ConstPtr& msg){
    if (msg->data){
        mutex = 1;
    }
}

void mutex_traj(){
    clock_t start_time = clock();
    while(1){
        if(mutex){
            break;
        }
        else if ((clock() - start_time) / CLOCKS_PER_SEC > 6.0)
            break;
    }
    mutex = 0;
}

const std::vector<std::string> listNamedPoses(moveit::planning_interface::MoveGroupInterface &group){
    // list of all stored pose name in SRDF 
    const std::vector<std::string> namedTargets = group.getNamedTargets();
    std::cout<<"stored position in SRDF: "<<std::endl;
    for(int i = 0; i < namedTargets.size(); i++){
        std::cout<<i<<": ";
        std::cout<<namedTargets[i]<<std::endl;
    }

    return namedTargets;
}

void gotoNamedTarget(moveit::planning_interface::MoveGroupInterface &group, std::string target, bool constraint_on){

    ROS_INFO("TASK: Go to %s pose", target.c_str());

    // get joint values of stored pose by name
    std::vector<double> group_variable_values;
    std::map<std::string, double> positions = group.getNamedTargetValues(target);
    for (std::map<std::string, double> ::iterator it=positions.begin(); it!=positions.end(); ++it){
      std::cout << it->first << " => " << it->second << '\n';
      group_variable_values.push_back(it->second);
    }
 
    // get constraints
    if(constraint_on) getConstraint(group, target);

    // plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    group.setJointValueTarget(group_variable_values);
    //moveit_msgs::MoveItErrorCodes success = 
    group.plan(my_plan);
    //compensate_slark(my_plan);
    /*
    sleep(5);

    ros::NodeHandle node_handle;
    ros::Publisher pub_8 = node_handle.advertise<std_msgs::Float64>("/finalasm_position_controller_8/command", 1, true);
    closeGrabber(pub_8,gripper_close_value_paw);
    sleep(5);
    */

    // set planning time to default
    group.setPlanningTime(5.0);
    group.clearPathConstraints();

    // visualization
    //ROS_INFO("Visualizing plan home (joint space goal) %s",success == moveit_msgs::MoveItErrorCodes::SUCCESS?"":"FAILED");
    //sleep(5.0);    
 
    // execution
    //ROS_INFO("Execution if plan successfully");
    //if(success == moveit_msgs::MoveItErrorCodes::SUCCESS) 
    group.execute(my_plan);
    mutex_traj();
    
    // show current Joints (the real robot joints, not the planned joints)
    std::vector<double> currentJoints = group.getCurrentJointValues();
    /*
    std::cout<< "current joint values:"<<std::endl;
    for(size_t i = 0; i<currentJoints.size(); i++) std::cout<< currentJoints[i]<<" ";
    std::cout<<std::endl;
    */
}

void addObject2Scene(moveit::planning_interface::MoveGroupInterface &group, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, ros::Publisher &collision_object_publisher){
    /* Define the object message */
    moveit_msgs::CollisionObject object;

    /* The header must contain a valid TF frame */
    object.header.frame_id = group.getPlanningFrame();
    /* The id of the object */
    object.id = "box";

    /* A default pose */
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x =  0.5;
    pose.position.y =  0.5;
    pose.position.z = -0.10;//-0.123;//-0.13;//-0.1;//-0.07

    /* Define a box to be attached */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.9;
    primitive.dimensions[1] = 0.9;
    primitive.dimensions[2] = 0.05;

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);


    /* An attach operation requires an ADD */
    object.operation = object.ADD;

    /* Publish and sleep (to view the visualized results) */
    collision_object_publisher.publish(object);

    std::vector<moveit_msgs::CollisionObject> collision_objects;  
    collision_objects.push_back(object);  

    // Now, let's add the collision object into the world
    ROS_INFO("Add an object into the world");  
    planning_scene_interface.addCollisionObjects(collision_objects);

    ros::WallDuration sleep_time(1.0);
    sleep_time.sleep();

}

void getConstraint(moveit::planning_interface::MoveGroupInterface &group, std::string target){
    ROS_INFO("get constraints");

    // switch wont take string
    int targetNum = 0;
    if(target == "home") targetNum = 0;
    else if(target == "holding") targetNum = 1;
    else targetNum = -1;

    // avoid cross init
    moveit_msgs::OrientationConstraint ocm; 
    moveit_msgs::Constraints test_constraints;
    switch(targetNum){
        case 0: // no constraints
          break;

        case 1: // hoding position  
          ocm.link_name = "link_8";  
          ocm.header.frame_id = "base_link";
          ocm.orientation.z = 1.0;
          ocm.absolute_x_axis_tolerance = 0.1; //0.1
          ocm.absolute_y_axis_tolerance = 0.1; //0.1
          ocm.absolute_z_axis_tolerance = 0.5; //0.1
          ocm.weight = 1.0;

          test_constraints.orientation_constraints.push_back(ocm);  
          group.setPathConstraints(test_constraints);
          group.setPlanningTime(5.0);
          break;

        default: // no constraints
          break;
    }
}
