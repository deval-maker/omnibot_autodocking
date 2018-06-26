#ifndef OMNIBOT_NAV_SRC_TRACKMODEL_H_
#define OMNIBOT_NAV_SRC_TRACKMODEL_H_

#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <string.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

enum track_model_errors_e
{
	TRACK_MODEL_SUCCESS,
	TRACK_MODEL_ERROR_GET_POSITION,
};

struct state_var_s
{
	double x;
	double y;
	double theta;

};

struct is_tracked_s
{
	bool x;
	bool y;
	bool theta;
};

class track_model {

private:

	ros::NodeHandle *node;
	ros::ServiceClient gazebo_model_state_client;
	ros::Publisher send_velo_pub;

	std::string goal_model;
	geometry_msgs::Pose model_position;

	std::string tracker_model;
	geometry_msgs::Pose tracker_position;

	geometry_msgs::Twist vel_to_tracker;
	geometry_msgs::Twist zero_velo;

	state_var_s tracker_state;
	state_var_s model_state;
	state_var_s goal_state;

	track_model_errors_e pose_to_state(geometry_msgs::Pose *pose, state_var_s *state);

public:

	track_model(std::string goal_model, std::string tracker_model, ros::NodeHandle* nodeH);
	~track_model();

	state_var_s tracking_thresholds;

	geometry_msgs::Twist velocity_upper_thresholds;
	geometry_msgs::Twist velocity_lower_thresholds;

	geometry_msgs::Pose goal;

	track_model_errors_e get_position(std::string model_name, geometry_msgs::Pose *model_pose);
	track_model_errors_e get_model_position();
	track_model_errors_e get_tracker_position();
	track_model_errors_e get_goal_position();
	track_model_errors_e compute_tracking_velocities();
	track_model_errors_e filter_tracking_velocities();
	track_model_errors_e send_tracker_velocities();
	track_model_errors_e get_all_positions();
};

#endif /* OMNIBOT_NAV_SRC_TRACKMODEL_H_ */
