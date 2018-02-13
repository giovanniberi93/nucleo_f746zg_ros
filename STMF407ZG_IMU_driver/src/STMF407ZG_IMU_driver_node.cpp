#include "ros/ros.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Imu.h"
#include <ctime>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>

#define NODE_NAME "nucleof746zg_driver"

using boost::asio::ip::udp;

std::string frame_id;
int udp_port;
// magnetometer
float mag_x_center;
float mag_y_center;
float mag_z_center;
float mag_x_width_correction;
float mag_y_width_correction;
float mag_z_width_correction;
// gyro
float gyro_x_center;
float gyro_y_center;
float gyro_z_center;
// acc
float acc_x_center;
float acc_y_center;
float acc_z_center;
float acc_x_width_correction;
float acc_y_width_correction;
float acc_z_width_correction;

void get_yaml_params(ros::NodeHandle Handle){
  Handle.param<int>("/nucleof746zg_driver/udp_port", udp_port, 7878);
  Handle.param<std::string>("/nucleof746zg_driver/frame_id", frame_id, "imu");
  // magnetometer calibration parameter
  Handle.param<float>("/nucleof746zg_driver/mag_x_center", mag_x_center, 0);
  Handle.param<float>("/nucleof746zg_driver/mag_y_center", mag_y_center, 0);
  Handle.param<float>("/nucleof746zg_driver/mag_z_center", mag_z_center, 0);
  Handle.param<float>("/nucleof746zg_driver/mag_x_width_correction", mag_x_width_correction, 1);
  Handle.param<float>("/nucleof746zg_driver/mag_y_width_correction", mag_y_width_correction, 1);
  Handle.param<float>("/nucleof746zg_driver/mag_z_width_correction", mag_z_width_correction, 1);

  // gyro calibration parameter
  Handle.param<float>("/nucleof746zg_driver/gyro_x_center", gyro_x_center, 0);
  Handle.param<float>("/nucleof746zg_driver/gyro_y_center", gyro_y_center, 0);
  Handle.param<float>("/nucleof746zg_driver/gyro_z_center", gyro_z_center, 0);

  // accelerometer calibration parameter
  Handle.param<float>("/nucleof746zg_driver/acc_x_center", acc_x_center, 0);
  Handle.param<float>("/nucleof746zg_driver/acc_y_center", acc_y_center, 0);
  Handle.param<float>("/nucleof746zg_driver/acc_z_center", acc_z_center, 0);
  Handle.param<float>("/nucleof746zg_driver/acc_x_width_correction", acc_x_width_correction, 1);
  Handle.param<float>("/nucleof746zg_driver/acc_y_width_correction", acc_y_width_correction, 1);
  Handle.param<float>("/nucleof746zg_driver/acc_z_width_correction", acc_z_width_correction, 1);

  ROS_INFO("Magnetometer center: [%2.2f; %2.2f; %2.2f]", mag_x_center, mag_x_center, mag_z_center);
  ROS_INFO("Magnetometer scale : [%2.2f; %2.2f; %2.2f]", mag_x_width_correction, mag_y_width_correction, mag_z_width_correction);

  ROS_INFO("Gyroscope center: [%2.2f; %2.2f; %2.2f]", gyro_x_center, gyro_x_center, gyro_z_center);

  ROS_INFO("Accelerometer center: [%2.2f; %2.2f; %2.2f]", acc_x_center, acc_x_center, acc_z_center);
  ROS_INFO("Accelerometer scale : [%2.2f; %2.2f; %2.2f]", acc_x_width_correction, acc_y_width_correction, acc_z_width_correction);
}


int main(int argc, char *argv[])
{
  // Init ROS components
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle Handle;
  get_yaml_params(Handle);
  ros::Publisher mag_pub = Handle.advertise<sensor_msgs::MagneticField>("f746zg_imu/mag", 1);
  ros::Publisher imu_pub = Handle.advertise<sensor_msgs::Imu>("f746zg_imu/data_raw", 1);

  sensor_msgs::MagneticField mag_measure_msg;
  sensor_msgs::Imu imu_measure_msg;

  std::vector<double> mag_measure(3);
  std::vector<double> gyro_measure(3);
  std::vector<double> acc_measure(3);
  bool firstMeasure = true;
  try
  {
    boost::asio::io_service io_service;
    udp::socket socket(io_service, udp::endpoint(udp::v4(), 7878));
    ROS_INFO("Ready to go, listen on port %d", udp_port);
    while(ros::ok())
    {
      boost::array<char, 350> recv_buf;
      udp::endpoint remote_endpoint;
      boost::system::error_code error;
      socket.receive_from(boost::asio::buffer(recv_buf), remote_endpoint, 0, error);

      if (error && error != boost::asio::error::message_size)
        throw boost::system::system_error(error);
      if(firstMeasure){
        ROS_INFO("Started streaming!");
        firstMeasure = false;
      }

      // build an std::string with the content of the UDP packet
      std::string received_data(std::string(reinterpret_cast<const char*>(recv_buf.data()), recv_buf.size()));
      // parse string and get data
      std::stringstream(received_data) >> mag_measure.at(0)  >> mag_measure.at(1)  >> mag_measure.at(2) 
                                       >> acc_measure.at(0)  >> acc_measure.at(1)  >> acc_measure.at(2)
                                       >> gyro_measure.at(0) >> gyro_measure.at(1) >> gyro_measure.at(2);
      // prepare and send magnetic field data
      // [milliGauss]
      ros::Time pkt_timestamp = ros::Time::now();
      mag_measure_msg.header.frame_id   = frame_id;
      mag_measure_msg.header.stamp.sec  = pkt_timestamp.sec;
      mag_measure_msg.header.stamp.nsec = pkt_timestamp.nsec;
      mag_measure_msg.magnetic_field.x  = ((mag_measure.at(0)/1000) - mag_x_center) * mag_x_width_correction;
      mag_measure_msg.magnetic_field.y  = ((mag_measure.at(1)/1000) - mag_y_center) * mag_y_width_correction;
      mag_measure_msg.magnetic_field.z  = ((mag_measure.at(2)/1000) - mag_z_center) * mag_z_width_correction;

      // prepare and publish gyro data
      // [rad/s]
      imu_measure_msg.header.frame_id    = frame_id;
      imu_measure_msg.header.stamp.sec   = pkt_timestamp.sec;
      imu_measure_msg.header.stamp.nsec  = pkt_timestamp.nsec;
      imu_measure_msg.angular_velocity.x = (gyro_measure.at(0) * (M_PI/180)/1000) - gyro_x_center;
      imu_measure_msg.angular_velocity.y = (gyro_measure.at(1) * (M_PI/180)/1000) - gyro_y_center;
      imu_measure_msg.angular_velocity.z = (gyro_measure.at(2) * (M_PI/180)/1000) - gyro_z_center;

      // prepare and publish accelerometer data
      // [m/s^2]
      imu_measure_msg.linear_acceleration.x = ((acc_measure.at(0)*9.81/1000) - acc_x_center) * acc_x_width_correction;
      imu_measure_msg.linear_acceleration.y = ((acc_measure.at(1)*9.81/1000) - acc_y_center) * acc_y_width_correction;
      imu_measure_msg.linear_acceleration.z = ((acc_measure.at(2)*9.81/1000) - acc_z_center) * acc_z_width_correction;

      mag_pub.publish(mag_measure_msg);
      imu_pub.publish(imu_measure_msg);
    }
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}