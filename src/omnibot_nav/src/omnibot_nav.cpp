#include <ros/ros.h>
#include "track_model.h"

int main (int argc, char** argv) {

	ros::init(argc,argv,"omnibot_nav");
	ros::NodeHandle n;

	track_model nav("table", "omnibot", &n);

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
