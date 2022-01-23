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

#include <moveit_visual_tools/moveit_visual_tools.h>

std::vector<double> current_joint_values;

void joint_state_handler(const sensor_msgs::JointState::ConstPtr& msg);
const std::vector<std::string> listNamedPoses(moveit::planning_interface::MoveGroupInterface &group);
void gotoNamedTarget(moveit::planning_interface::MoveGroupInterface &group, std::string target, bool constraint_on);
void addObject2Scene(moveit::planning_interface::MoveGroupInterface &group, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, ros::Publisher &collision_object_publisher);
void getConstraint(moveit::planning_interface::MoveGroupInterface &group, std::string target);
void plan_pose_goal(moveit::planning_interface::MoveGroupInterface &ee_group, moveit::core::JointModelGroup joint_model_group, moveit_visual_tools::MoveItVisualTools &visual_tools);

namespace rvt = rviz_visual_tools;

int main(int argc, char **argv)
{

    /*****************************************************************
    *                         Arm initialization                     *
    *****************************************************************/
    // ros initialization
    ros::init(argc, argv, "arm_kinematics");
    ros::NodeHandle node_handle;  

    ros::Subscriber sub_multi_joint_states = node_handle.subscribe<sensor_msgs::JointState>("/joint_states", 1, joint_state_handler);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "left_arm";
    moveit::planning_interface::MoveGroupInterface group_l_arm("PLANNING_GROUP");
    group_l_arm.setPlannerId("BKPIECEkConfigDefault");

    // We will use the planning_scene_interface class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 

    const moveit::core::JointModelGroup* joint_model_group = group_l_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    /*****************************************************************
    *                         ROS info output                        *
    *****************************************************************/
    // get the joint names to verify if loaded successfully 
    const std::vector<std::string> &l_joint_names = group_l_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_l_arm.getName())->getJointModelNames();

    // get the link names to verify if loaded successfully 
    const std::vector<std::string> &l_link_names = group_l_arm.getCurrentState()->getRobotModel()->getJointModelGroup(group_l_arm.getName())->getLinkModelNames();

    ROS_INFO("Model frame: %s", group_l_arm.getCurrentState()->getRobotModel()->getModelFrame().c_str());
    ROS_INFO("Reference Planning frame: %s", group_l_arm.getPlanningFrame().c_str());
    ROS_INFO("EndEffectorLink simulator: %s", group_l_arm.getEndEffectorLink().c_str());

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

    /***************************
	*       Visualization      *
    ***************************/
    moveit_visual_tools::MoveItVisualTools visual_tools("root");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    /*****************************************************************
    *         Specify pre-defined initial joint configuration        *
    *****************************************************************/
    // namedTargets stores the names of pre-defined poses(joint values)
    // select the name (ex. "home"), it is defined in srdf file
    std::string target = ""; 
    int targetNum = 0;
    std::cout<<"select target pose above: "; std::cin >> targetNum;
    if(targetNum == 0) target = "home";
    else target = "home";
    
    gotoNamedTarget(group_l_arm, target + "_l", 0);

    
    int terminate = 1;
    while (terminate) {
    	plan_pose_goal(group_l_arm, *joint_model_group, visual_tools);
        std::cout<<"Do you want to continue? 1(Yes), 0(No):"<<std::endl;
        std::cin>>terminate;
    }

    ros::spin();
    return 0;
}

void plan_pose_goal(moveit::planning_interface::MoveGroupInterface ee_group, moveit::core::JointModelGroup joint_model_group, moveit_visual_tools::MoveItVisualTools visual_tools){
    double x,y,z,angle;

    std::cout<<"Type the x position (unit m):"<<std::endl;std::cin>>x;
    std::cout<<"Type the y position (unit m):"<<std::endl;std::cin>>y;
    std::cout<<"Type the z position (unit m):"<<std::endl;std::cin>>z;
    std::cout<<"Type the rotation angle (degree) of wrist:"<<std::endl;std::cin>>angle;

    // set the target pose 
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    tf::Quaternion qat = tf::createQuaternionFromRPY(0, -M_PI, M_PI + angle / 180 * M_PI);
    qat.normalize();
    pose.orientation.x = qat.x();
    pose.orientation.y = qat.y();
    pose.orientation.z = qat.z();
    pose.orientation.w = qat.w(); 

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode success;

    // plan the target pose goal
    ee_group.setPoseTarget(pose);
    success = (ee_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // visualize plans
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(pose, "pose1");
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
}

void joint_state_handler(const sensor_msgs::JointState::ConstPtr& msg)
{
    current_joint_values.clear();
    for(std::size_t i = 0; i < msg->position.size(); ++i) {
        current_joint_values.push_back(msg->position[i]);
    }
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
    
    // show current Joints (the real robot joints, not the planned joints)
    std::vector<double> currentJoints = group.getCurrentJointValues();
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
