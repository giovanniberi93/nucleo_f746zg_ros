#include "ros/ros.h"
#include <signal.h>
#include <fstream>
#include <sensor_msgs/Imu.h>

#define NODE_NAME "calibrate_gyror"
#define NODE_RATE 200

void sigint_handler(int);
volatile sig_atomic_t sig_flag = 0;

class ROSnode {
private:
	void imu_measure_callback(const sensor_msgs::Imu::ConstPtr& measure);

	ros::NodeHandle Handle;
	ros::Subscriber imuSub;

	struct avgGyroValue_t{
		double x;
		double y;
		double z;
	} avgGyroValue;

	struct avgAccValue_t{
		double x;
		double y;
		double z;
	} avgAccValue;

	int num_measurement;
	// ctrl+c handler
	void (*sig_handler)(int);
	// log file
	std::ofstream logFile;
public:
	void prepare();
	void runContinuously();
	~ROSnode();
};

//-------------------------------------------------
//-------------------------------------------------

// recognize ctrl-c
void sigint_handler(int p){
	sig_flag = 1;
}

void ROSnode::prepare(){
	// callback to ctrl+c
	sig_handler = signal (SIGINT, sigint_handler);
	logFile.open("log");
	
	imuSub = Handle.subscribe("/f746zg_imu/data_raw", 1, &ROSnode::imu_measure_callback, this);
	num_measurement = 0;

}


void ROSnode::runContinuously(){
    ros::Rate LoopRate(NODE_RATE);
    ROS_INFO("Node %s running periodically (T=%.2fs, f=%dHz).", ros::this_node::getName().c_str(), 1.0/NODE_RATE, NODE_RATE);

    while (ros::ok()){
        ros::spinOnce();
        LoopRate.sleep();
    }
}

void ROSnode::imu_measure_callback(const sensor_msgs::Imu::ConstPtr& data){

	logFile << data->linear_acceleration.x << "," << data->linear_acceleration.y << "," << data->linear_acceleration.z << std::endl;

	num_measurement++;

	avgGyroValue.x = (avgGyroValue.x * (num_measurement-1) + data->angular_velocity.x) / num_measurement;
	avgGyroValue.y = (avgGyroValue.y * (num_measurement-1) + data->angular_velocity.y) / num_measurement;
	avgGyroValue.z = (avgGyroValue.z * (num_measurement-1) + data->angular_velocity.z) / num_measurement;

	avgAccValue.x = (avgAccValue.x * (num_measurement-1) + data->linear_acceleration.x) / num_measurement;
	avgAccValue.y = (avgAccValue.y * (num_measurement-1) + data->linear_acceleration.y) / num_measurement;
	avgAccValue.z = (avgAccValue.z * (num_measurement-1) + data->linear_acceleration.z) / num_measurement;
	
	std::cout << "-----------------------------" << std::endl;
	ROS_INFO("Gyro avg x: %f", avgGyroValue.x);
	ROS_INFO("Gyro avg y: %f", avgGyroValue.y);
	ROS_INFO("Gyro avg z: %f", avgGyroValue.z);
	std::cout << std::endl;
	ROS_INFO("Acc avg x: %f", avgAccValue.x);
	ROS_INFO("Acc avg y: %f", avgAccValue.y);
	ROS_INFO("Acc avg z: %f", avgAccValue.z);

	std::cout << "-----------------------------" << std::endl;
	if(!sig_flag)
		// magic to clear the console
		std::cout << "\033[2J\033[1;1H";
	else {
		logFile.close();
		exit(0);
	}

}

ROSnode::~ROSnode(){

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