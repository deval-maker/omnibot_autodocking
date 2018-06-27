#include "goal_pub.h"

int main(int argc, char **argv)
{

	ros::init(argc, argv, "talker");

	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<geometry_msgs::Pose>("/goal", 2);

	ros::Rate loop_rate(1);
	int count = 0;

	while (ros::ok())
	{
		geometry_msgs::Pose msg;

		msg.position.x = 1.0;
		msg.position.y = 2.0;
		msg.position.z = 3.0;
		msg.orientation.w = 1;

		ROS_INFO("Sending");;

		chatter_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}

  return 0;
}
