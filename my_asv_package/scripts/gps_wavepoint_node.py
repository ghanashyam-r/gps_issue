#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

geometry_msgs::PointStamped target_gps_point;
bool new_gps_received = false;

// Callback function for receiving GPS coordinates
void gpsCallback(const geometry_msgs::PointStamped::ConstPtr& gps_msg) {
    target_gps_point = *gps_msg;
    new_gps_received = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_navigation");
    ros::NodeHandle nh;

    // Subscribe to GPS input topic
    ros::Subscriber gps_sub = nh.subscribe("/user_gps_input", 1, gpsCallback);

    MoveBaseClient ac("/move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    if (argc != 3) {
        ROS_ERROR("Usage: gps_navigation <latitude> <longitude>");
        return 1;
    }

    target_gps_point.point.x = atof(argv[1]); // Latitude
    target_gps_point.point.y = atof(argv[2]); // Longitude
    target_gps_point.header.frame_id = "map"; // Assuming the GPS points are in the map frame

    // Create a goal to send to move_base
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map"; // Assuming the GPS points are in the map frame
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = target_gps_point.point.x;
    goal.target_pose.pose.position.y = target_gps_point.point.y;
    goal.target_pose.pose.orientation.w = 1.0; // Assuming no rotation needed

    // Send the goal
    ac.sendGoal(goal);

    // Wait for the result
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("ASV has reached the target GPS point");
    } else {
        ROS_ERROR("ASV was unable to reach the target GPS point");
    }

    return 0;
}
