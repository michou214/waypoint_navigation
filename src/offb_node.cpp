/* Modes 1: Takes off and lands*/


/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse> 
using namespace Eigen;
//namespace apriltags_ros;

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>      // Mavros local pose
#include <geometry_msgs/TransformStamped.h> // Mavros local pose
#include <geometry_msgs/PoseArray.h>        // Tag detection
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
//#include <gazebo_msgs/ModelStates.h>
#include <apriltags_ros/AprilTagDetectionArray.h>


#include "std_msgs/String.h"
#include <poll.h>
#include <sstream>
#include <vector>
#include <cmath>

#include "offb_node.h"


#define HEIGHT 1.75f

// No modification are necessary but you can adjuste as you want

#define TOL 0.1f
#define POS_ACCEPT_RAD 0.2f // From Aerial robots
#define POS_ACCEPT_Z   0.1f
#define Z_OFFSET 0.3f


// Global variables
int idx  = 0; // index to select a point in the list of points 

bool armed = true;
bool traj_done = false;
bool takeoff_done = false;
bool landing_done = false;
bool landing_in_progress = false;


mavros_msgs::State          current_state;
geometry_msgs::PoseStamped  est_local_pos;      // Mavros local pose
//gazebo_msgs::ModelStates    true_local_pos;     // Gazebo true local pose
//geometry_msgs::PoseArray    APtag_est_pos;      // Tag detection
apriltags_ros::AprilTagDetectionArray APtag_est_pos;// Tag detection


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
//ros::Subscriber true_local_pos_sub = nh.subscribe<gazebo_msgs::ModelStates>
//("gazebo/model_states", 10, true_local_pos_cb);

// The node subscribe to Topic "/tag_detections_pose", 10 msgs in buffer before deleting
//ros::Subscriber APtag_est_pos_sub = nh.subscribe<geometry_msgs::PoseArray>
//("tag_detections_pose", 10, APtag_est_pos_cb);
// The node subscribe to Topic "/tag_detections_pose", 10 msgs in buffer before deleting
ros::Subscriber APtag_est_pos_sub = nh.subscribe<apriltags_ros::AprilTagDetectionArray>
("tag_detections", 10, APtag_est_pos_cb);

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


    Vector3f pos     (0.0f, 0.0f, 0.0f);
    Vector3f goal_pos(5.0f, 5.0f, 5.0f); 
    Vector3f takeoff (0.0f, 0.0f, HEIGHT);
    Vector3f landing1(0.0f, 0.0f, HEIGHT);
    Vector3f landing2(0.0f, 0.0f, -0.15f);
    Vector3f landing[2] = {landing1, landing2};
    int size_land = sizeof(landing)/sizeof(landing[0]);

    // IMPORTANT TO MENTION
    //send a few setpoints before starting
    for(int i = 50; ros::ok() && i > 0; --i){
        local_pos_pub.publish(conversion_to_msg(pos));
        ros::spinOnce();
        rate.sleep();
    }   

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
        if( current_state.mode  != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
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


        // Current location in vector form.
        pos = conversion_to_vect(est_local_pos);
        //ROS_INFO("POS =[%f, %f, %f]", pos(0), pos(1), pos(2));

        if(arming_client.call(arm_cmd) && is_goal_reached(goal_pos, pos, POS_ACCEPT_RAD)){
            if(idx==0 && !takeoff_done){
                takeoff_done = true;
                idx = 0; // To reset for waypoints array
                traj_done = true;
                landing_in_progress = true;
                //ROS_INFO("COND 9");
            }
            else if(takeoff_done && traj_done && idx == size_land){
                landing_done = true;
                //ROS_INFO("COND 5");
            }
            else if( takeoff_done && traj_done && landing_done){
                idx = idx;
                //ROS_INFO("COND 6");
                if(!current_state.armed)
                    arm_cmd.request.value = false;
            }
            else{
                idx++;
                //ROS_INFO("COND 7 idx = %d", idx);
            }
        }

        if(!current_state.armed && landing_done){
            arm_cmd.request.value = false;
            // Disarmed drone
            ROS_INFO("Vehicle disarmed");
            //if( !arming_client.call(arm_cmd) && !arm_cmd.response.success)
             //   ROS_INFO("Vehicle disarmed");
        }
        else{   
            if(takeoff_done){
                if(traj_done && idx == size_land){
                    //ROS_INFO("COND 1");
                    landing_done = true;
                    current_state.armed = false;
                }

                else if (traj_done && landing_in_progress){ 
                    //ROS_INFO("COND 2");
                    goal_pos = landing[idx];
                }
                else
                    goal_pos = waypoints[idx];
                    //Vector3f v = goal_pos;
                    //ROS_INFO("GOAL2=[%f, %f, %f], idx=%d", v(0), v(1), v(2), idx);
            }
            else
                goal_pos = takeoff;

  
            //Vector3f v = goal_pos;
            //ROS_INFO("GOAL1=[%f, %f, %f], idx=%d", v(0), v(1), v(2), idx);
            local_pos_pub.publish(conversion_to_msg(goal_pos));
        }

        // Needed or your callbacks would never get called
        ros::spinOnce();
        // Sleep for the time remaining to have 10Hz publish rate
        rate.sleep();
    }

    return 0;
}


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
//    ROS_INFO("Est. local pos=[%f, %f, %f]", est_pos->pose.position.x,
//                                            est_pos->pose.position.y,
//                                            est_pos->pose.position.z);
}


// To check is the drone is at the "right" place
bool is_goal_reached(Vector3f a, Vector3f b, float tol)
{
    if( (a-b).norm() < tol && fabs(a(2)-b(2)) < POS_ACCEPT_Z)   
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

