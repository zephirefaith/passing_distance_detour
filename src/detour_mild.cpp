//ros headers
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>

void GetDist2PersonCB( const geometry_msgs::PoseStamped::ConstPtr& msg ){

}

int main(int argc, char **argv){

    ros::init(argc, argv, "detour_mild");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    ros::Subscriber closest_person_sub = nh.subscribe("people_tracker/pose", 1000, GetDist2PersonCB());
    
    while( ros::ok() ){
        loop_rate.sleep();
        ros::spinOnce();
    }
}
