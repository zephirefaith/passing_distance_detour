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
#include <tf/transform_listener.h>

//ros messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

bool detour_flag = false;
geometry_msgs::PoseWithCovarianceStamped robot_pose;
float start_heading = 0.0;
float observed_lateral_distance = 0.0;

void DetourCB( const std_msgs::Bool::ConstPtr &msg){
    if ( msg->data ){
        start_heading = tf::getYaw(robot_pose.pose.pose.orientation);
        detour_flag = true;
    } else {
        detour_flag = false;
    }
}

void UpdatePositionCB ( const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg){
    robot_pose = *msg;
}

void UpdateDistanceCB ( const geometry_msgs::PoseStampedConstPtr &msg){
    observed_lateral_distance = abs( robot_pose.pose.pose.position.y - msg->pose.position.y );
}

int main(int argc, char **argv){

    ros::init(argc, argv, "detour_mild");
    ros::NodeHandle nh;
    ros::Rate loop_rate(35);
    ros::Time start_time;
    int fsm_state = 0;

    ros::Subscriber detour_signal_sub = nh.subscribe("detour_signal", 1000, DetourCB);
    ros::Subscriber robot_position_sub = nh.subscribe("amcl_pose", 1000, UpdatePositionCB);
    ros::Subscriber person_position_sub = nh.subscribe("people_tracker/pose", 1000, UpdateDistanceCB);

    ros::Publisher twist_velocity_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1000);

    geometry_msgs::Twist velocity;
    float desired_lateral_distance = 2; //metres
    
    while( ros::ok() ){

        if( detour_flag ){
            if( fsm_state == 0 ){
                start_time = ros::Time::now();
                velocity.linear.x = 0.6;
                velocity.angular.z = 0.6;
                ROS_INFO("moving out of the way routine");
                fsm_state = 1;
            }
            if ( ( fsm_state == 1) && ( ros::Time::now() - start_time >= ros::Duration( 2 ) ) ){
                fsm_state = 2;
                start_time = ros::Time::now();
                velocity.angular.z = 0.0;
            }
            if ( ( fsm_state == 2) && ( ros::Time::now() - start_time >= ros::Duration( 3 ) ) ){
                fsm_state = 3;
                start_time = ros::Time::now();
            }
            if( fsm_state == 4 || fsm_state == 3 ){
                if ( fsm_state == 3 ){
                    velocity.angular.z = -1.2;
                    velocity.linear.x = 0.4;
                }
                if ( ( fsm_state == 3 ) && ( ros::Time::now() - start_time >= ros::Duration( 1.1 ) ) ){
                    velocity.angular.z = 0.0;
                    velocity.linear.z = 0.45;
                    fsm_state = 4;
                    start_time = ros::Time::now();
                }
                ROS_INFO("walking straight routine");
            }
            
            twist_velocity_pub.publish(velocity);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
}
