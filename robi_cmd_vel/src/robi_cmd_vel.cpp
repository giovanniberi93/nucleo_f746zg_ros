#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
// boost components
#include <ctime>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>

#define NODE_NAME "robi_vel_controller"
#define NODE_RATE 100

using boost::asio::ip::udp;

class ROSnode {
private:
	void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& vel);
	void getYamlParams();

	ros::NodeHandle Handle;
	ros::Subscriber velSub;
	int destinationPort;
	std::string destinationIp;
	// UDP socket variables
	std::unique_ptr<udp::endpoint> destination;
	boost::asio::io_service io_service;
	udp::socket mySocket;
public:
	ROSnode();
	void prepare();
	void runContinuously();
};

//-------------------------------------------------
//-------------------------------------------------


ROSnode::ROSnode() : io_service(),
					 mySocket(io_service, udp::v4())
{

}

void ROSnode::prepare(){
	getYamlParams();
	velSub = Handle.subscribe("robi_cmd_vel", 1, &ROSnode::cmd_vel_callback, this);
	destination.reset(new udp::endpoint(boost::asio::ip::address::from_string(destinationIp), destinationPort));
}

void ROSnode::runContinuously(){
    ros::Rate LoopRate(NODE_RATE);
    ROS_INFO("Node %s running periodically (T=%.2fs, f=%dHz).", ros::this_node::getName().c_str(), 1.0/NODE_RATE, NODE_RATE);

    while (ros::ok()){
        ros::spinOnce();
        LoopRate.sleep();
    }
}

void ROSnode::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& vel){
	size_t sentBytes;
	boost::array<char, 350> send_buf;
	mySocket.send_to(boost::asio::buffer(send_buf), *(destination.get()));
	ROS_INFO("");
}

void ROSnode::getYamlParams(){
	Handle.param<int>("/robi_cmd_vel/stm_destination_port", destinationPort, 100);
	Handle.param<std::string>("/robi_cmd_vel/stm_destination_ip", destinationIp, "192.168.1.11");
} 


//-------------------------------------------------
//-------------------------------------------------


int main(int argc, char *argv[])
{
	ros::init(argc, argv, NODE_NAME);
	ROSnode myNode;
	myNode.prepare();
	myNode.runContinuously();
	return 0;
}