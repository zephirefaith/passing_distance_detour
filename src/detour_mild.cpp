/*  
 *  detour_mild.cpp
 *  Purpose: Subscribes to closest person position topic and caculates distance to that person
 *
 *  @author Priyam Parashar
 *  @version 0.0 8/27/16 
 *
 */

//ros headers
#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/client/simple_action_client.h>

//ros messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <move_base_msgs/MoveBaseAction.h>

//typedefs
typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> MoveBaseClient;

class PersonRobotTracker{
    
    public:
        float person_to_robot_distance;
        ros::Time person_time_stamp;

        PersonRobotTracker(){
            person_to_robot_distance = -1.0;
            person_time_stamp = ros::Time();
        }

        void GetDist2PersonCB( const geometry_msgs::PoseStampedConstPtr &msg ){
            geometry_msgs::PoseStamped person_pose = *msg;
            person_to_robot_distance = (float) sqrt( ( robot_pose_x_ - person_pose.pose.position.x )*( robot_pose_x_ - person_pose.pose.position.x ) + ( robot_pose_y_ - person_pose.pose.position.y )*( robot_pose_y_ - person_pose.pose.position.y ) );
            person_time_stamp = person_pose.header.stamp;
            ROS_INFO("Distance from person: %f m", person_to_robot_distance);
        }
        
        void UpdateSelfPositionCB( const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg ){
            robot_pose_x_ = msg->pose.pose.position.x; 
            robot_pose_y_ = msg->pose.pose.position.y; 
            robot_orientation_ = msg->pose.pose.orientation;
        }

        geometry_msgs::Quaternion get_orientation(){ return robot_orientation_; }

    private:
        float robot_pose_x_;
        float robot_pose_y_;
        geometry_msgs::Quaternion robot_orientation_;
};

int main(int argc, char **argv){

    ros::init(argc, argv, "detour_mild");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    MoveBaseClient ac("move_base", true);
    PersonRobotTracker tracker;

    ros::Subscriber closest_person_sub = nh.subscribe("people_tracker/pose", 1000, &PersonRobotTracker::GetDist2PersonCB, &tracker);
    ros::Subscriber robot_position_sub = nh.subscribe("amcl_pose", 1000, &PersonRobotTracker::UpdateSelfPositionCB, &tracker);
    
    //wait for the action server to come up
    ROS_INFO("Waiting for the move_base action server to come up");
    while ( !ac.waitForServer(ros::Duration(5.0)) && ros::ok()) {
        ROS_INFO("...");
        ros::spinOnce();
    }
    ROS_INFO("Client connected!");

    move_base_msgs::MoveBaseGoal jeeves_goal;
    jeeves_goal.target_pose.header.frame_id = "map";
    jeeves_goal.target_pose.header.stamp = ros::Time::now();
    jeeves_goal.target_pose.pose.position.x = -3.00;
    jeeves_goal.target_pose.pose.position.y = 3.2;
    jeeves_goal.target_pose.pose.orientation.x = 0.0;
    jeeves_goal.target_pose.pose.orientation.y = 0.0;
    jeeves_goal.target_pose.pose.orientation.z = 0.99;
    jeeves_goal.target_pose.pose.orientation.w = 0.1;

    //send the goal and add a feedback callback
    ac.sendGoal(jeeves_goal);
    ROS_INFO("Chasing waypoint: %f, %f", jeeves_goal.target_pose.pose.position.x, jeeves_goal.target_pose.pose.position.y);

    while( ros::ok() ){
        if( !tracker.person_time_stamp.is_zero() ){
            ros::Duration time_elapsed = ros::Time::now() - tracker.person_time_stamp;
            ROS_INFO("Time elapsed since last sighting of a person: %f", time_elapsed.toSec());
        }
        if( ( tracker.person_to_robot_distance < 4.0 ) && ( tracker.person_to_robot_distance > 0.0 ) ){
            ac.cancelGoal();
            ROS_INFO("Stopping because the person is too close!");
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
}
