#ifndef OMNIBOT_NAV_SRC_TRACKMODEL_H_
#define OMNIBOT_NAV_SRC_TRACKMODEL_H_

#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <string.h>
#include <geometry_msgs/Pose.h>

enum track_model_errors_e
{
	TRACK_MODEL_SUCCESS,
	TRACK_MODEL_ERROR_GET_POSITION,
};

class track_model {
public:

	track_model(std::string goal_model, std::string tracker_model, ros::NodeHandle* nodeH);
	~track_model();

	ros::NodeHandle *node;
	ros::ServiceClient gazebo_model_state_client;
	ros::Publisher send_velo_pub;

	std::string goal_model;
	geometry_msgs::Pose goal_position;

	std::string tracker_model;
	geometry_msgs::Pose tracker_position;

	geometry_msgs::Pose tracking_thresholds;

	geometry_msgs::Twist velocity_upper_thresholds;
	geometry_msgs::Twist velocity_lower_thresholds;

	geometry_msgs::Twist vel_to_tracker;

	geometry_msgs::Twist zero_velo;

	bool is_tracked;

	track_model_errors_e get_position(std::string model_name, geometry_msgs::Pose *model_pose);
	track_model_errors_e get_goal_position();
	track_model_errors_e get_tracker_position();
	track_model_errors_e check_tracking();
	track_model_errors_e set_goal_position();
	track_model_errors_e compute_tracking_velocities();
	track_model_errors_e filter_velocities();
	track_model_errors_e send_tracker_velocities(geometry_msgs::Twist);
};

#endif /* OMNIBOT_NAV_SRC_TRACKMODEL_H_ */
