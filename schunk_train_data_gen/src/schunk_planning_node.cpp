#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc , char **argv) {

    //Initialize as ROS node
    ros::init(argc ,argv , "schunk_planning_node") ;
    ros::NodeHandle n ;

    ros::AsyncSpinner spinner(1) ;

    spinner.start() ;

    static const std::string PLANNING_GROUP_ARM = "arm" ;
    //static const std::string PLANNING_GROUP_EE = "end_effector" ;
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP_ARM) ;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const robot_state::JointModelGroup* joint_model_group = 
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);


    geometry_msgs::Pose target_pose1;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.5;



    move_group.setPoseTarget(target_pose1);


    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group.move();
    
    ros::shutdown();    

    return 0 ;
}
