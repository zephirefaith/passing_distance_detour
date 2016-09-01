/*  
 *  fsm_control.cpp
 *  Purpose: Subscribes to closest person position topic and caculates distance to that person, taking care of FSM
 *
 *  @author Priyam Parashar
 *  @version 0.0 8/27/16 
 *
 */

//ros headers
#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

//ros messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Bool.h>

//typedefs
typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> MoveBaseClient;

class PersonRobotTracker{
    
    public:
        bool is_pursuing_goal;
        bool detour_behavior;
        float person_to_robot_distance;
        ros::Time person_time_stamp;

        PersonRobotTracker(){
            person_to_robot_distance = -1.0;
            person_time_stamp = ros::Time();
            is_pursuing_goal = false;
            detour_behavior = false;
        }

        void GetDist2PersonCB( const geometry_msgs::PoseStampedConstPtr &msg ){
            geometry_msgs::PoseStamped person_pose = *msg;
            person_to_robot_distance = (float) sqrt( ( robot_pose_x_ - person_pose.pose.position.x )*( robot_pose_x_ - person_pose.pose.position.x ) + ( robot_pose_y_ - person_pose.pose.position.y )*( robot_pose_y_ - person_pose.pose.position.y ) );
            person_time_stamp = person_pose.header.stamp;
            ROS_INFO("Dist 4m person: %f", person_to_robot_distance);
        }
        
        void UpdateSelfPositionCB( const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg ){
            robot_pose_.header = msg->header;
            robot_pose_.pose = msg->pose.pose;
            robot_pose_x_ = robot_pose_.pose.position.x;
            robot_pose_y_ = robot_pose_.pose.position.y;
        }

        double get_orientation(){ return tf::getYaw(robot_pose_.pose.orientation); }
        float get_pose_x() { return robot_pose_x_; }
        float get_pose_y() { return robot_pose_y_; }

    private:
        float robot_pose_x_;
        float robot_pose_y_;
        geometry_msgs::PoseStamped robot_pose_;
};

int main(int argc, char **argv){

    ros::init(argc, argv, "detour_mild");
    ros::NodeHandle nh;
    ros::Rate loop_rate(40);
    ros::Duration time_elapsed;
    MoveBaseClient ac("move_base", true);
    PersonRobotTracker tracker;

    int fsm_state = 0;

    ros::Subscriber closest_person_sub = nh.subscribe("people_tracker/pose", 1000, &PersonRobotTracker::GetDist2PersonCB, &tracker);
    ros::Subscriber robot_position_sub = nh.subscribe("amcl_pose", 1000, &PersonRobotTracker::UpdateSelfPositionCB, &tracker);
    ros::Publisher detour_pub = nh.advertise<std_msgs::Bool>("detour_signal", 1000);
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    
    //wait for the action server to come up
    ROS_INFO("Waiting for the move_base action server to come up");
    while ( !ac.waitForServer(ros::Duration(5.0)) && ros::ok()) {
        ROS_INFO("...");
        ros::spinOnce();
    }
    ROS_INFO("Client connected!");

    ros::Duration( 10.0 ).sleep();
    
    move_base_msgs::MoveBaseGoal jeeves_goal, original_goal;
    jeeves_goal.target_pose.header.frame_id = "map";
    jeeves_goal.target_pose.header.stamp = ros::Time::now();
    jeeves_goal.target_pose.pose.position.x = 3.900;
    jeeves_goal.target_pose.pose.position.y = 3.000;
    jeeves_goal.target_pose.pose.orientation.x = 0.0;
    jeeves_goal.target_pose.pose.orientation.y = 0.0;
    jeeves_goal.target_pose.pose.orientation.z = 0.99;
    jeeves_goal.target_pose.pose.orientation.w = 0.1;
    original_goal = jeeves_goal;

    //send the goal and add a feedback callback
    ac.sendGoal(jeeves_goal);
    tracker.is_pursuing_goal = true;
    ROS_INFO("Chasing waypoint: %f, %f", jeeves_goal.target_pose.pose.position.x, jeeves_goal.target_pose.pose.position.y);

    while( ros::ok() ){
        if( !tracker.person_time_stamp.is_zero() ){
            time_elapsed = ros::Time::now() - tracker.person_time_stamp;
            //ROS_INFO("Time elapsed since last sighting of a person: %f", time_elapsed.toSec());
        }
        if( ( tracker.person_to_robot_distance < 4.0 ) && ( tracker.person_to_robot_distance > 0.0 ) && ( fsm_state == 0 ) ){
            ac.cancelAllGoals();
            //jeeves_goal.target_pose.pose.position.x = tracker.get_pose_x();
            //jeeves_goal.target_pose.pose.position.y = tracker.get_pose_y();
            //ac.sendGoal(jeeves_goal);
            //ac.waitForResult();
            ROS_INFO("Stopping because the person is too close!");
            tracker.is_pursuing_goal = false;
            tracker.detour_behavior = true;
            fsm_state = 1;
        }
        if ( tracker.detour_behavior && ( fsm_state == 1 ) ){
            if ( time_elapsed > ros::Duration( 2, 0 ) ) {
                tracker.detour_behavior = false;
                ROS_INFO("I think it is safe to end the detour now");
                fsm_state = 2;
            }
        }
        if ( fsm_state == 2 ){
            geometry_msgs::Twist velocity;
            for( int i = 0 ; i < 10 ; i++ ){
                velocity_pub.publish(velocity);
            }
            ROS_INFO("We're going again!!");
            ac.sendGoal(original_goal);
            fsm_state = 3;
        }
        
        std_msgs::Bool detour_topic;
        detour_topic.data = tracker.detour_behavior;
        detour_pub.publish( detour_topic );
        loop_rate.sleep();
        ros::spinOnce();
    }
}
