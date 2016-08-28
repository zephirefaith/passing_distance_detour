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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class PersonRobotTracker{
    
    public:
        float person_to_robot_distance;
        ros::Time person_time_stamp;

        PersonRobotTracker(){
            person_to_robot_distance = 0.0;
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
        }

    private:
        float robot_pose_x_;
        float robot_pose_y_;
};

int main(int argc, char **argv){

    ros::init(argc, argv, "detour_mild");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    PersonRobotTracker tracker;

    ros::Subscriber closest_person_sub = nh.subscribe("people_tracker/pose", 1000, &PersonRobotTracker::GetDist2PersonCB, &tracker);
    ros::Subscriber robot_position_sub = nh.subscribe("amcl_pose", 1000, &PersonRobotTracker::UpdateSelfPositionCB, &tracker);
    
    while( ros::ok() ){
        if( !tracker.person_time_stamp.is_zero() ){
            ros::Duration time_elapsed = ros::Time::now() - tracker.person_time_stamp;
            ROS_INFO("Time elapsed since last sighting of a person: %f", time_elapsed.toSec());
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
}
