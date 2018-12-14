/* File         : offb_node.h
 * Author       : Michael Perret
 * Version      : 0.0.1
 * Description  : Callback function for topics subscription and other for vector calculation
 */

using namespace Eigen;

void state_cb(const mavros_msgs::State::ConstPtr& msg);
void est_local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& est_pos);
bool is_goal_reached(Vector3f a, Vector3f b, float tol);
geometry_msgs::PoseStamped conversion_to_msg(Vector3f a);
Vector3f conversion_to_vect(geometry_msgs::PoseStamped a);


