#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

class OdomToPath {
protected:
	ros::NodeHandle nh;
	ros::Subscriber sub;
	ros::Publisher pub;
	nav_msgs::Path path;
	double lastx, lasty;
	const double epsilon;  //< minimum distance between subsequent path points

public:
	OdomToPath() : lastx(-1.), lasty(-1.), epsilon(0.01) {
		sub = nh.subscribe("odom", 10, &OdomToPath::odomCallback, this);
		pub = nh.advertise<nav_msgs::Path>("odom_path", 10, false);
	}

	void odomCallback(const nav_msgs::OdometryConstPtr& odom) {
		if (hypot(odom->pose.pose.position.x - lastx, odom->pose.pose.position.y - lasty) >= epsilon) {
			path.header = odom->header;
			geometry_msgs::PoseStamped ps;
			ps.header = odom->header;
			ps.pose = odom->pose.pose;
			path.poses.push_back(ps);
			pub.publish(path);
			lastx = odom->pose.pose.position.x;
			lasty = odom->pose.pose.position.y;
		}
	}
};

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "odom_to_path");
	OdomToPath otp;
	ros::spin();
	return 0;
}
