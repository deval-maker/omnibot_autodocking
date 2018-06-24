#include <ros/ros.h>
#include "track_model.h"

int main (int argc, char** argv)
{
	ros::init(argc,argv,"omnibot_nav");
	ros::NodeHandle n;

	track_model nav("table", "omnibot", &n);

	ros::Rate loop_rate(10);

	track_model_errors_e status = TRACK_MODEL_SUCCESS;


	if(TRACK_MODEL_SUCCESS == status)
	{
		status = nav.check_tracking();
	}

	while (ros::ok())
	{

		if(!nav.is_tracked)
		{
			if(TRACK_MODEL_SUCCESS == status)
			{
				status = nav.set_goal_position();
			}

			if(TRACK_MODEL_SUCCESS == status)
			{
				status = nav.compute_tracking_velocities();
			}

			if(TRACK_MODEL_SUCCESS == status)
			{
				status = nav.send_tracker_velocities(nav.vel_to_tracker);
			}

			if(TRACK_MODEL_SUCCESS == status)
			{
				status = nav.check_tracking();
			}
		}

		loop_rate.sleep();
	}

	return 0;
}
