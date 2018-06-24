#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <string.h>
#include <geometry_msgs/Pose.h>

int main (int argc, char** argv) {

	ros::init(argc,argv,"omnibot_nav");
	ros::NodeHandle n;

//	Pseudo Code:
//
//	While ros shutdown
//
//		get_goal();
//		get bot position
//
//		should bot move ?
//
//	    get intermediate goal (eliminate one (shortest dist))
//			(if already at the intermediate pos, give the actual goal)
//
//		go_to_goal (x, y, theta) (send velocities towards goal)
//			upper thresholds and lower thresholds to the output velos


	return 0;
}

class track_model
{
	private:

	public:

	ros::NodeHandle *node;
	ros::ServiceClient gazebo_model_state_client;

	std::string goal_model;
	geometry_msgs::Pose goal_position;

	std::string tracker_model;
	geometry_msgs::Pose tracker_position;

	geometry_msgs::Pose tracking_thresholds;

	geometry_msgs::Twist velocity_upper_thresholds;
	geometry_msgs::Twist velocity_lower_thresholds;

	geometry_msgs::Twist vel_to_tracker;

	bool is_tracked;

	enum track_model_errors_e
	{
		TRACK_MODEL_SUCCESS,
		TRACK_MODEL_ERROR_GET_GOAL,
	};

	track_model(std::string goal_model, std::string tracker_model, ros::NodeHandle* nodeH){


	}

	~track_model(){
		// send zero velocities

	}

	track_model_errors_e get_goal_position(){

		track_model_errors_e status = TRACK_MODEL_SUCCESS;


		return status;
	}

	track_model_errors_e get_tracker_position(){

		track_model_errors_e status = TRACK_MODEL_SUCCESS;


		return status;
	}

	track_model_errors_e set_is_tracked(){
		track_model_errors_e status = TRACK_MODEL_SUCCESS;

		// is_tracked = true if goal-tracker < threshold

		return status;
	}

	track_model_errors_e set_goal_position(){

		track_model_errors_e status = TRACK_MODEL_SUCCESS;

		// get intermediate goal (eliminate one (shortest dist))
		// (if already at the intermediate pos, give the actual goal)

		return status;
	}

	track_model_errors_e compute_tracking_velocities(){

		track_model_errors_e status = TRACK_MODEL_SUCCESS;

		// go_to_goal (x, y, theta) (send velocities towards goal)

		return status;
	}

	track_model_errors_e filter_velocities(){

		track_model_errors_e status = TRACK_MODEL_SUCCESS;

		// upper thresholds and lower thresholds to the output velos


		return status;
	}

	track_model_errors_e send_tracker_velocities(){

		track_model_errors_e status = TRACK_MODEL_SUCCESS;


		return status;
	}

};

