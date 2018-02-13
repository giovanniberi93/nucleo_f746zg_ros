#include "ros/ros.h"
#include <signal.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>

#define NODE_NAME "calibrate_magnetometer"
#define NODE_RATE 200

void sigint_handler(int);
volatile sig_atomic_t sig_flag = 0;

typedef struct triple_t{
	double x;
	double y;
	double z;
} triple_t;

class ROSnode {
private:
	void mag_measure_callback(const sensor_msgs::MagneticField::ConstPtr& measure);
	void acc_measure_callback(const sensor_msgs::Imu::ConstPtr& measure);
	void printValues();

	ros::NodeHandle Handle;
	// subscribers
	ros::Subscriber magSub;
	ros::Subscriber accSub;
	triple_t mag_max;
	triple_t mag_min;
	triple_t acc_max;
	triple_t acc_min;

	// ctrl+c handler
	void (*sig_handler)(int);
public:
	void prepare();
	void runContinuously();
	~ROSnode();
};

//-------------------------------------------------
//-------------------------------------------------

void sigint_handler(int p){
	sig_flag = 1;
}

void ROSnode::prepare(){
	// callback to ctrl+c
	// sig_handler = signal (SIGINT, std::bind(&ROSnode::sigint_handler, this,_1));
	
	magSub = Handle.subscribe("/f746zg_imu/mag", 1, &ROSnode::mag_measure_callback, this);
	accSub = Handle.subscribe("/f746zg_imu/data_raw", 1, &ROSnode::acc_measure_callback, this);

	mag_max.x = -1000;
	mag_max.y = -1000;
	mag_max.z = -1000;
	acc_max.x = -1000;
	acc_max.y = -1000;
	acc_max.z = -1000;

	mag_min.x = 1000;
	mag_min.y = 1000;
	mag_min.z = 1000;
	acc_min.x = 1000;
	acc_min.y = 1000;
	acc_min.z = 1000;
}


void ROSnode::runContinuously(){
    ros::Rate LoopRate(NODE_RATE);
    ROS_INFO("Node %s running periodically (T=%.2fs, f=%dHz).", ros::this_node::getName().c_str(), 1.0/NODE_RATE, NODE_RATE);

    while (ros::ok()){
        ros::spinOnce();
        printValues();
        LoopRate.sleep();
    }
}

void ROSnode::acc_measure_callback(const sensor_msgs::Imu::ConstPtr& data){

	if(data->linear_acceleration.x > acc_max.x)
		acc_max.x = data->linear_acceleration.x;
	if(data->linear_acceleration.y > acc_max.y)
		acc_max.y = data->linear_acceleration.y;
	if(data->linear_acceleration.z > acc_max.z)
		acc_max.z = data->linear_acceleration.z;

	if(data->linear_acceleration.x < acc_min.x)
		acc_min.x = data->linear_acceleration.x;
	if(data->linear_acceleration.y < acc_min.y)
		acc_min.y = data->linear_acceleration.y;
	if(data->linear_acceleration.z < acc_min.z)
		acc_min.z = data->linear_acceleration.z;
}


void ROSnode::mag_measure_callback(const sensor_msgs::MagneticField::ConstPtr& data){

	if(data->magnetic_field.x > mag_max.x)
		mag_max.x = data->magnetic_field.x;
	if(data->magnetic_field.y > mag_max.y)
		mag_max.y = data->magnetic_field.y;
	if(data->magnetic_field.z > mag_max.z)
		mag_max.z = data->magnetic_field.z;

	if(data->magnetic_field.x < mag_min.x)
		mag_min.x = data->magnetic_field.x;
	if(data->magnetic_field.y < mag_min.y)
		mag_min.y = data->magnetic_field.y;
	if(data->magnetic_field.z < mag_min.z)
		mag_min.z = data->magnetic_field.z;
}

void ROSnode::printValues(){
	std::cout << "-----------------------------" << std::endl;
	ROS_INFO("Magnetometer x: [%2.4f; %2.4f]", mag_min.x, mag_max.x);
	ROS_INFO("             y: [%2.4f; %2.4f]", mag_min.y, mag_max.y);
	ROS_INFO("             z: [%2.4f; %2.4f]", mag_min.z, mag_max.z);
	
	ROS_INFO("Accelerometer x: [%2.4f; %2.4f]", acc_min.x, acc_max.x);
	ROS_INFO("              y: [%2.4f; %2.4f]", acc_min.y, acc_max.y);
	ROS_INFO("              z: [%2.4f; %2.4f]", acc_min.z, acc_max.z);

	std::cout << "-----------------------------" << std::endl;
	if(!sig_flag)
		// magic to clear the console
		std::cout << "\033[2J\033[1;1H";
	else
		exit(0);


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