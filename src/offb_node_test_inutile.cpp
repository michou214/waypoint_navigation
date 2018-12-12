/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse> 
using namespace Eigen;

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>      // Mavros local pose
#include <geometry_msgs/TransformStamped.h> // Mavros local pose
#include <geometry_msgs/PoseArray.h>        // Tag detection
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <gazebo_msgs/ModelStates.h>


#include "std_msgs/String.h"
#include <poll.h>
#include <sstream>
#include <vector>

#include "offb_node.h"

// From Aerial robots
#define POS_ACCEPT_RAD 0.2f
#define AP_POS_ACCEPT 0.1f
#define AP_POS_ACCEPT_X 0.1f
#define AP_POS_ACCEPT_Y 0.1f
#define HEIGHT 1.0f
#define DESIRED_ID 2


// Global variables
//int idx  = 0; // index to select a point in the list of points 
int n_AP = 0; //To know how many AP are detected
int n_model=0;
//int AP_id=10;

//bool armed = true;
//bool traj_done = false;
bool AP_detected  = false;
//bool AP_centered  = false;
//bool takeoff_done = false;
//bool landing_done = false;
//bool AP_in_verification = false;

mavros_msgs::State          current_state;
geometry_msgs::PoseStamped  est_local_pos;      // Mavros local pose
gazebo_msgs::ModelStates    true_local_pos;     // Gazebo true local pose
geometry_msgs::PoseArray    APtag_est_pos;      // Tag detection



int main(int argc, char **argv)
{


    // Initialize ROS => where we specify the name of our node
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // The node subscribe to Topic "mavros/state", 10 msgs in buffer before deleting
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

// The node subscribe to Topic "mavros/local_position/pose", 10 msgs in buffer before deleting
ros::Subscriber est_local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
("mavros/local_position/pose", 10, est_local_pos_cb);

// The node subscribe to Topic "gazebo/model_states", 10 msgs in buffer before deleting
ros::Subscriber true_local_pos_sub = nh.subscribe<gazebo_msgs::ModelStates>
("gazebo/model_states", 10, true_local_pos_cb);

// The node subscribe to Topic "/tag_detections_pose", 10 msgs in buffer before deleting
ros::Subscriber APtag_est_pos_sub = nh.subscribe<geometry_msgs::PoseArray>
("tag_detections_pose", 10, APtag_est_pos_cb);

    // The node publish the commanded local position        
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // This creates a client for the arming status of the drone
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    // This creates a client for the mode set on the drone
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection (to  established between MAVROS and the autopilot)
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    Vector3f pos      (0.0f, 0.0f, 0.0f);
    Vector3f AP_pos   (0.0f, 0.0f, 0.0f);
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;

    //send a few setpoints before starting
    for(int i = 50; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // Current location in vector form.
    pos = conversion_to_vect(est_local_pos);




    // Create msgs_structure of the type mavros_msgs::SetMode
    mavros_msgs::SetMode offb_set_mode;
    // Fix the param 'custom_mode'
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // Create msgs_structure of the type mavros_msgs::CommandBool
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // Get the time info
    ros::Time last_request = ros::Time::now();


    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                ROS_INFO("Offboard enabled");
            last_request = ros::Time::now();
        } 
        else{
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                    ROS_INFO("Vehicle armed");
                last_request = ros::Time::now();
            }
        }

        if(AP_detected){
            AP_pos(0) = APtag_est_pos.poses[0].position.x;
            AP_pos(1) = APtag_est_pos.poses[0].position.y;
            AP_pos(2) = APtag_est_pos.poses[0].position.z;
        }

        ROS_INFO("Pos =[%f, %f, %f]", pos(0),pos(1),pos(2));
        ROS_INFO("APP =[%f, %f, %f]", AP_pos(0),AP_pos(1),AP_pos(2));            if(n_model>0)
        ROS_INFO("TLP =[%f, %f, %f]", true_local_pos.pose[5].position.x,true_local_pos.pose[5].position.y,true_local_pos.pose[5].position.z);
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}



// ------------------------------------------------------------------------

// Callback which will save the current state of the autopilot
// Like 'arming', 'disarming', 'takeoff', 'landing', 'Offboard flags'
//mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// Callback which will save the estimated local position of the autopilot
//geometry_msgs::PoseStamped est_local_pos;
void est_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& est_pos){
    est_local_pos = *est_pos;
    //ROS_INFO("EST =[%f, %f, %f]", est_pos->pose.position.x,
    //                                        est_pos->pose.position.y,
    //                                       est_pos->pose.position.z);
}

// Callback which will save the estimated local position of the autopilot
//gazebo_msgs::ModelStates true_local_pos;
void true_local_pos_cb(const gazebo_msgs::ModelStates::ConstPtr& true_pos){
    true_local_pos = *true_pos;
    n_model = true_pos->pose.size();
    if(n_model>0){
    //ROS_INFO("TRP =[%f, %f, %f]", true_pos->pose[5].position.x,
    //                              true_pos->pose[5].position.y,
    //                              true_pos->pose[5].position.z);
    }
}

// Callback which will save the estimated local position of the Apriltag
//geometry_msgs::PoseArray APtag_est_pos;
void APtag_est_pos_cb(const geometry_msgs::PoseArray::ConstPtr& AP_est_pos){
    //ROS_INFO("CALLBACK AP");
    APtag_est_pos = *AP_est_pos;
    n_AP = AP_est_pos->poses.size();
    //ROS_INFO("n=%d", n_AP);
    if(n_AP>0){
        AP_detected = true;
        //AP_in_verification = true;
            //ROS_INFO("APP =[%f, %f, %f]",  AP_est_pos->poses[0].position.x,
            //                               AP_est_pos->poses[0].position.y,
            //                               AP_est_pos->poses[0].position.z);
    }
    else
        AP_detected = false;

}


// To check is the drone is at the "right" place
bool is_goal_reached(Vector3f a, Vector3f b, float tol)
{
    if( (a-b).norm() < tol )   
        return true;
    else                            
        return false;
}

// To convert a vector into msg format
geometry_msgs::PoseStamped conversion_to_msg(Vector3f a){
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = a(0);
    msg.pose.position.y = a(1);
    msg.pose.position.z = a(2);
    return msg;
}

// To convert a msg format in vector
Vector3f conversion_to_vect(geometry_msgs::PoseStamped a){
    Vector3f v;
    v(0) = a.pose.position.x;
    v(1) = a.pose.position.y;
    v(2) = a.pose.position.z;
    return v;
}