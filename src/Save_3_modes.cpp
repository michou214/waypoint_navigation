/* There is the original offb_node at the end => L. 260*/



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



// Parameters to modify
#define MODE 0
#define MODE_TL 0      // Take off, stay 5 secondes in the air, lands
#define MODE_TPL 1      // Take off, go to position [2,2,2], comes back to starting pose, lands
#define MODE_PROJECT 2  // Do the semester project
// --- Apriltag parameters
#define DESIRED_ID 2
// --- Waypoint generation parameters
#define NB_CYCLE 3
#define W 1 // Spacing in the trajectory generation, see drawing
#define L 1 // Spacing in the trajectory generation, see drawing
#define ALPHA 20 // Every ALPHA[°] you take the point on the Archimed spiral
#define SQUARE 1
#define RECT_HORI 2
#define RECT_VERT 3
#define SPIRAL 4  // Parameters for the Archimed spiral (a,b) are in WP_generation()
#define SEARCH_SHAPE SPIRAL // Modify to choose the type of search 
#define HEIGHT 1.75f

// No modification are necessary but you can adjuste as you want
#define LANDING 5
#define DEFAULT 0
#define TOL 0.1f
#define POS_ACCEPT_RAD 0.2f // From Aerial robots
#define POS_ACCEPT_Z   0.1f
#define AP_SIZE 0.2f
#define AP_POS_ACCEPT   0.1f
#define AP_POS_ACCEPT_X 0.1f
#define AP_POS_ACCEPT_Y 0.1f
#define OFFSET_CAM_X 0
#define OFFSET_CAM_Y 0
#define SP_SIZE_WIDTH  0.8f  // SP=Solar Panel of size 1.6x0.8[m]
#define SP_SIZE_LENGTH 1.6f // SP=Solar Panel of size 1.6x0.8[m]
#define SP_SIZE_HEIGHT 0.8f // SP=Solar Panel of size 1.6x0.8[m]
#define Z_OFFSET 0.3f
#define SIZE_CLEAN_WP 6


// Global variables
int idx  = 0; // index to select a point in the list of points 
int n_AP = 0; //To know how many AP are detected
int AP_id=10;
int n_model=0;

bool chrono_start = false;
bool stop  = false;
bool skip  = false;
bool armed = true;
bool traj_done = false;
bool AP_detected  = false;
bool AP_centered  = false;
bool AP_verified  = false;
bool takeoff_done = false;
bool landing_done = false;
bool cleaning_done = false;
bool landing_in_progress = false;
bool cleaning_in_progress = false;



mavros_msgs::State          current_state;
geometry_msgs::PoseStamped  est_local_pos;      // Mavros local pose
//gazebo_msgs::ModelStates    true_local_pos;     // Gazebo true local pose
//geometry_msgs::PoseArray    APtag_est_pos;      // Tag detection
apriltags_ros::AprilTagDetectionArray APtag_est_pos;// Tag detection


int main(int argc, char **argv)
{
    int size_wp = 0;

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


    Vector3f pos        (0.0f, 0.0f, 0.0f);
    Vector3f goal_pos   (5.0f, 5.0f, 5.0f); 
    Vector3f takeoff    (0.0f, 0.0f, HEIGHT);
    Vector3f AP_pos     (0.0f, 0.0f, 0.0f);
    Vector3f AP_pos_save(0.0f, 0.0f, 0.0f);
    Vector3f cleaning_waypoints[SIZE_CLEAN_WP];


    // -------------------------
    // Waypoint generation steps
    // -------------------------
    // Initialization of the number of waypoints = size_wp
    if(MODE == MODE_PROJECT){
        if(SEARCH_SHAPE == SQUARE || SEARCH_SHAPE == RECT_HORI || SEARCH_SHAPE == RECT_VERT)
            size_wp = NB_CYCLE*4+1; // 4 points per cycle + the starting point
        if(SEARCH_SHAPE == SPIRAL)
            size_wp = ((NB_CYCLE-1)*(360/ALPHA)+1)+1; // 360/alpha points per cycle + starting point
        //ROS_INFO("Size_wp=%d",size_wp);
    }
    if (MODE == MODE_TPL)
        size_wp = 3;

    // Creation of the table to store the different positions
    Vector3f waypoints[size_wp];
    for(int p=0; p<size_wp; p++){
        waypoints[p](0) = 0.0f;
        waypoints[p](1) = 0.0f,
        waypoints[p](2) = 0.0f;
    }


    // Waypoints generation depending of the MODE
    if(MODE == MODE_PROJECT) 
        WP_generation(takeoff, NB_CYCLE, W, L, ALPHA, waypoints, size_wp, SEARCH_SHAPE);

    if(MODE == MODE_TPL)
        for(int p=0; p<size_wp; p++){
            waypoints[p](0) =  1.0f + 0.5f*p;
            waypoints[p](1) = -1.0f + 0.5f*p;
            waypoints[p](2) = HEIGHT;
        }


    // Display to check
    for(int p=0; p<size_wp; p++)
        ROS_INFO("WP[%d] =[%f, %f, %f]", p, waypoints[p](0),waypoints[p](1),waypoints[p](2));


    // To ensure there is no problem for landing, you must set more waypoint than for landing
    Vector3f landing1( 0.0f,  0.0f, HEIGHT);
    Vector3f landing2( 0.0f,  0.0f, -0.15f);
    Vector3f landing[2] = {landing1, landing2};
    int size_land = sizeof(landing)/sizeof(landing[0]);

    for(int p=0; p<size_land; p++)
        ROS_INFO("L[%d] =[%f, %f, %f]", p, landing[p](0),landing[p](1),landing[p](2));


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


        if(!skip && AP_detected && !AP_verified){

            AP_pos(0) = APtag_est_pos.detections[0].pose.pose.position.x;
            AP_pos(1) = APtag_est_pos.detections[0].pose.pose.position.y;
            AP_pos(2) = HEIGHT;

            goal_pos = to_center_pose(pos, AP_pos, OFFSET_CAM_X, OFFSET_CAM_Y);

            if(is_AP_centered(AP_pos, AP_POS_ACCEPT_X, AP_POS_ACCEPT_Y)){
                if(check_id(DESIRED_ID)){
                    AP_verified = true;
                    AP_pos_save = goal_pos; 
                    cleaning_path(AP_pos_save, cleaning_waypoints, AP_id);
                }
                else
                    if (takeoff_done)
                        skip = true;
            }
            local_pos_pub.publish(conversion_to_msg(goal_pos));
        }
        else{
            if(arming_client.call(arm_cmd) && is_goal_reached(goal_pos, pos, POS_ACCEPT_RAD)){
                if(AP_verified && !cleaning_in_progress){
                    idx=0;
                    cleaning_in_progress = true;
                    ROS_INFO("COND 8");
                } 
                else if(idx==0 && !takeoff_done){
                    takeoff_done = true;
                    idx = 0; // To reset for waypoints array
                    if(MODE == MODE_TL){
                        traj_done = true;
                        landing_in_progress = true;
                    }
                    ROS_INFO("COND 9");
                }
                else if(takeoff_done && cleaning_done && !landing_in_progress){
                    traj_done = true;
                    idx = 0;
                }
                else if(takeoff_done && idx == (size_wp-1)){
                    traj_done = true;
                    idx = 0;
                    landing_in_progress = true;
                    ROS_INFO("COND 4");
                }
                else if(takeoff_done && traj_done && idx == size_land){
                    landing_done = true;
                    ROS_INFO("COND 5");
                }
                else if( takeoff_done && traj_done && landing_done){
                    idx = idx;
                    ROS_INFO("COND 6");
                    if(!current_state.armed)
                        arm_cmd.request.value = false;
                }
                else{
                    idx++;
                    ROS_INFO("COND 7 idx = %d", idx);
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
                        ROS_INFO("COND 1");
                        landing_done = true;
                        current_state.armed = false;
                    }

                    else if (traj_done && landing_in_progress){ 
                        ROS_INFO("COND 2");
                        goal_pos = landing[idx];
                    }

                    else if (AP_verified){
                        if(cleaning_in_progress){
                            if(idx < SIZE_CLEAN_WP)
                                goal_pos = cleaning_waypoints[idx];
                            else{
                                ROS_INFO("COND 3");
                                cleaning_done = true;
                                cleaning_in_progress = false;
                                landing_in_progress = true;
                                idx = 0;
                            }
                        }
                        if(cleaning_done && is_goal_reached(goal_pos, pos, POS_ACCEPT_RAD))
                             traj_done = true;  
                    }
                    else {
                        if(MODE == MODE_PROJECT || MODE == MODE_TPL)
                            goal_pos = waypoints[idx];
                        Vector3f v = goal_pos;
                        ROS_INFO("GOAL2=[%f, %f, %f], idx=%d", v(0), v(1), v(2), idx);
                    }
                }
                else
                    goal_pos = takeoff;

      
                Vector3f v = goal_pos;
                ROS_INFO("GOAL1=[%f, %f, %f], idx=%d", v(0), v(1), v(2), idx);
                local_pos_pub.publish(conversion_to_msg(goal_pos));
            }
        }
        // Needed or your callbacks would never get called
        ros::spinOnce();
        // Sleep for the time remaining to have 10Hz publish rate
        rate.sleep();
    }

    return 0;
}


// ------------------------------------------------------------------------------------------------
void cleaning_path(Vector3f p, Vector3f *array, int id){

    Vector3f P0, P1, P2, P3, P4, P5;
    float H = SP_SIZE_HEIGHT+Z_OFFSET;

    if (id == 0){
        P0 << p(0)                      , p(1)                  , H;
        P1 << p(0) +AP_SIZE/2.0f        , p(1)+SP_SIZE_LENGTH   , H;
        P2 << P1(0)-SP_SIZE_WIDTH/2.0f  , P1(1)                 , H;
        P3 << P2(0)                     , p(1)-AP_SIZE          , H;
        P4 << P1(0)-SP_SIZE_WIDTH       , P3(1)                 , H;
        P5 << P4(0)                     , P1(1)                 , H;
    }
    else if (id == 1){
        P0 << p(0)                      , p(1)                  , H;
        P1 << p(0)-SP_SIZE_LENGTH       , p(1)-AP_SIZE/2.0f         , H;
        P2 << P1(0)                     , P1(1)+SP_SIZE_WIDTH/2.0f  , H;
        P3 << p(0)+AP_SIZE              , P2(1)                     , H;
        P4 << P3(0)                     , P1(1)+SP_SIZE_WIDTH       , H;
        P5 << P1(0)                     , P4(1)                     , H;
    }       
    else if (id == 2){
        P0 << p(0)                      , p(1)                  , H;
        P1 << p(0)+SP_SIZE_LENGTH       , p(1)-AP_SIZE/2.0f         , H;
        P2 << P1(0)                     , P1(1)+SP_SIZE_WIDTH/2.0f  , H;
        P3 << p(0)-AP_SIZE              , P2(1)                     , H;
        P4 << P3(0)                     , P1(1)+SP_SIZE_WIDTH       , H;
        P5 << P1(0)                     , P4(1)                     , H;
    }
    else{ // Just in case
        P0 << 0.0f, 0.0f, HEIGHT;
        P1 << 0.0f, 0.0f, HEIGHT;
        P2 << 0.0f, 0.0f, HEIGHT/2.0f;
        P3 << 0.0f, 0.0f, HEIGHT/2.0f;
        P4 << 0.0f, 0.0f, 0.0f;
        P5 << 0.0f, 0.0f, 0.0f;
    }

    Vector3f temp[6] = {P0, P1, P2, P3, P4, P5};
    for(int p=0; p<6; p++)
        array[p] = temp[p];
}   


void WP_generation(Vector3f p, int cycle, float w, float l, int angle, Vector3f *array, int size, int type){
    
    int i = 0; int m = 0;
    float a = 0.05; float b = 0.005; // Parameters for the spiral, found experimentaly
    Vector3f P0, P1, P2, P3;
    Vector4i k; k << 1,2,3,4;
    Vector4i z; z << 4,4,4,4;
    Vector4i k_modif; k_modif << 0,0,0,0;
    array[0]=p;

    while(i!=cycle){
        if(type == SQUARE){
            //ROS_INFO("SQUARE TRAJECTORY %d",i);
            if(i == 0)  P0 << p(0)          , p(1)+(i+1)*w  , p(2);  
            else        P0 << p(0)-i*l      , p(1)+(i+1)*w  , p(2);
                        P1 << p(0)+(i+1)*l  , P0(1)         , p(2);
                        P2 << P1(0)         , p(1)-(i+1)*w  , p(2);
                        P3 << p(0)-(i+1)*l  , P2(1)         , p(2);
        }
        else if (type == RECT_VERT){
            //ROS_INFO("RECT VERT TRAJECTORY");
            P0 << p(0)+(2*i)*w    , p(1)+l  , p(2);
            P1 << p(0)+(2*i+1)*w  , P0(1)   , p(2);
            P2 << P1(0)           , p(1)    , p(2);
            P3 << p(0)+(2*i+2)*w  , p(1)    , p(2);
        }
        else if (type == RECT_HORI){
            //ROS_INFO("RECT HORI TRAJECTORY");
            P0 << p(0)+l , p(1)+(2*i)*w     , p(2);
            P1 << P0(0)  , p(1)+(2*i+1)*w   , p(2);
            P2 << p(0)   , P1(1)            , p(2);
            P3 << p(0)   , p(1)+(2*i+2)*w   , p(2);
        }
        else if (type == SPIRAL){
            //ROS_INFO("SPIRAL TRAJECTORY");
            float x = (a+b*angle*m)*cos(m*angle*M_PI/180.0f);
            float y = (a+b*angle*m)*sin(m*angle*M_PI/180.0f);
            P0 << x, y, p(2);
        }
        else{ // Just in case
            //ROS_INFO("DEFAULT TRAJECTORY");
            P0 <<  1.9f,  1.9f, HEIGHT;
            P1 <<  1.9f, -1.9f, HEIGHT;
            P2 << -1.9f, -1.9f, HEIGHT;
            P3 << -1.9f,  1.9f, HEIGHT;
        }

        if (type == SPIRAL){
            array[m+1] = P0;
            if(m % (360/angle) == 0) i++;
            m++;
        }
        else{
            Vector3f temp[4] = {P0, P1, P2, P3};
            k_modif = i*z + k;
            for(int j=0; j<4; j++)
                array[int(k_modif(j))] = temp[j]; 
            i++;
        }
    }
}



Vector3f lands(Vector3f a, float H){
    Vector3f v(0.0f, 0.0f, 0.0f);
    v(0) = a(0);
    v(1) = a(1);
    v(2) = H-0.1f;
    return v;
}

Vector3f landing_on_SP(Vector3f a, int id){
    Vector3f v(0.0f, 0.0f, 0.0f);
    if(id==0){
        v(0) = a(0) - AP_SIZE/2.0f - SP_SIZE_WIDTH/2.0f;
        v(1) = a(1) + AP_SIZE/2.0f + SP_SIZE_LENGTH/2.0f;
        v(2) = a(2);
    }
    else if (id==1){
        v(0) = a(0) - AP_SIZE/2.0f - SP_SIZE_LENGTH/2.0f;
        v(1) = a(1) + AP_SIZE/2.0f + SP_SIZE_WIDTH/2.0f;
        v(2) = a(2);
    }
    else if (id==2){
        v(0) = a(0) + AP_SIZE/2.0f + SP_SIZE_LENGTH/2.0f;
        v(1) = a(1) - AP_SIZE/2.0f + SP_SIZE_WIDTH/2.0f;
        v(2) = a(2);
    }
    else
        ROS_INFO("Not a defined tags");
    return v;
}

Vector3f to_center_pose(Vector3f real_world, Vector3f camera_world, float offset_x, float offset_y){
    Vector3f target_pos(0.0f, 0.0f, 0.0f);
    target_pos(0) = real_world(0)-camera_world(1)+offset_y;
    target_pos(1) = real_world(1)-camera_world(0)+offset_x;
    target_pos(2) = camera_world(2);
    return target_pos;
}

// To check is the drone is at the centered with the AP
bool is_AP_centered(Vector3f a, float tol_x, float tol_y){
    if( fabs(a(0)) < tol_x   &&   fabs(a(1)) < tol_y)   return true;
    else                                                return false;                
} 

// To check if you found the right id
bool check_id(int id){
    if(AP_id == id)     return true;
    else                return false;
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

// Callback which will save the estimated local position of the autopilot
//gazebo_msgs::ModelStates true_local_pos;
//void true_local_pos_cb(const gazebo_msgs::ModelStates::ConstPtr& true_pos){
//   true_local_pos = *true_pos;
//   n_model = true_pos->pose.size();
    //if(n_model>0){
    //ROS_INFO("TRP =[%f, %f, %f]", true_pos->pose[5].position.x,
    //                              true_pos->pose[5].position.y,
    //                              true_pos->pose[5].position.z);
    //}
//}


// Callback which will save the estimated local position of the Apriltag
//geometry_msgs::PoseArray APtag_est_pos;
void APtag_est_pos_cb(const apriltags_ros::AprilTagDetectionArray::ConstPtr& AP_est_pos){
    APtag_est_pos = *AP_est_pos;
    n_AP = AP_est_pos->detections.size();

    //ROS_INFO("n=%d", n_AP);
    if (n_AP == 0){
        skip = false;
        AP_detected = false;
    }
    else{
        AP_id = AP_est_pos->detections[0].id;
        //ROS_INFO("ID=%d", AP_id);
        AP_detected = true;
        //AP_in_verification = true;
            //ROS_INFO("APtag est pos=[%f, %f, %f]",  AP_est_pos->.detections[0].pose.pose.position.x,
            //                                        AP_est_pos->.detections[0].pose.pose.position.y,
            //                                        AP_est_pos->.detections[0].pose.pose.position.z);
    }
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

