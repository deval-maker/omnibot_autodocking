#include "track_model.h"

int main (int argc, char** argv)
{
	ros::init(argc,argv,"omnibot_nav");
	ros::NodeHandle n;

	ROS_INFO("Waiting for gazebo model state service.");

	ros::service::waitForService("/gazebo/get_model_state", -1);
	track_model nav("table", "omnibot", &n);
	ros::Subscriber sub = n.subscribe("/goal", 2, &track_model::set_goal_position_cb, &nav);

	ros::Rate loop_rate(50);
	track_model_errors_e status = TRACK_MODEL_SUCCESS;

	ROS_INFO("Omnibot Nav node started.");

	while (ros::ok())
	{
		// Sense
		if(TRACK_MODEL_SUCCESS == status)
		{
			status = nav.get_tracker_position();
		}

		ros::spinOnce();

		// Plan
		if(TRACK_MODEL_SUCCESS == status)
		{
			status = nav.compute_tracking_velocities();
		}

		// Act
		if(TRACK_MODEL_SUCCESS == status)
		{
			status = nav.send_tracker_velocities();
		}

		loop_rate.sleep();
	}

	return 0;
}
