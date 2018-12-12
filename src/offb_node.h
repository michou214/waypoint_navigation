/* File         : offb_node.h
 * Author       : Michael Perret
 * Version      : 0.0.1
 * Description  : Callback function for topics subscription and other for vector calculation
 */

using namespace Eigen;

void state_cb(const mavros_msgs::State::ConstPtr& msg);
void est_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& est_pos);
//void true_local_pos_cb(const gazebo_msgs::ModelStates::ConstPtr& true_pos);
//void APtag_est_pos_cb(const geometry_msgs::PoseArray::ConstPtr& AP_est_pos);
void APtag_est_pos_cb(const apriltags_ros::AprilTagDetectionArray::ConstPtr& AP_est_pos);
bool is_goal_reached(Vector3f a, Vector3f b, float tol);
geometry_msgs::PoseStamped conversion_to_msg(Vector3f a);
Vector3f conversion_to_vect(geometry_msgs::PoseStamped a);
//bool is_goal_AP_centered(Vector3f a, Vector3f b, float tol_x, float tol_y);
bool is_AP_centered(Vector3f a, float tol_x, float tol_y);
bool check_id(int id);
Vector3f to_center_pose(Vector3f real_world, Vector3f camera_world, float offset_x, float offset_y);
Vector3f landing_on_SP(Vector3f a, int id);

Vector3f lands(Vector3f a, float H);
void WP_generation(Vector3f p, int cycle, float w, float l, int angle, Vector3f *array, int size, int type);
void cleaning_path(Vector3f p, Vector3f *array, int id);



