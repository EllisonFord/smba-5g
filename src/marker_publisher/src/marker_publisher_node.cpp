#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include <random>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ground_truth");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<visualization_msgs::MarkerArray>("ground_truth", 1);

  ros::Rate loop_rate(10);

  std::string bus       = "Bus";
  std::string truck     = "Truck";
  std::string motorbike = "Motorbike";

  while (ros::ok()){

    visualization_msgs::MarkerArray msg;

    int num_cars = 7;

    msg.markers.resize(num_cars);

    for (int i = 0; i < num_cars; i += 1){

	msg.markers[i].header.seq++;
	msg.markers[i].header.stamp = ros::Time::now();
	msg.markers[i].header.frame_id = "/base_link";

	msg.markers[i].id = i+1;

	msg.markers[i].type = i % 3; // Three types of vehicles will be shown

	msg.markers[i].pose.position.x = char(ros::Time::now().toNSec()); // get the time in nano-seconds and then reduce it to a char sized number
	msg.markers[i].pose.position.y = i*5;
	msg.markers[i].pose.position.z = 0;

	msg.markers[i].lifetime = ros::Duration(0.5);

	msg.markers[i].type = 1; // CUBE=1

	msg.markers[i].color.r = 0;

	msg.markers[i].color.g = 1.0f;

	msg.markers[i].color.b = 0;

	msg.markers[i].color.a = 1;

	msg.markers[i].scale.x = 3.0;

	msg.markers[i].scale.y = 2.0;

	msg.markers[i].scale.z = 2.0;

	if (i == 1){
	
		msg.markers[i].text = bus;
	}
	
	else if (i == 3){
	
		msg.markers[i].text = motorbike;
	}

	else if (i % 5 == 0){
	
		msg.markers[i].text = truck;
	}

	else {
	
		msg.markers[i].text = "Car";
	}	

	ROS_INFO("ID:%u\tX=%f\tY=%f", i+1, msg.markers[i].pose.position.x, msg.markers[i].pose.position.y);

    }


    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
