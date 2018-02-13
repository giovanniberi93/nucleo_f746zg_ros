#include <ctime>
#include <iostream>
#include <string>
#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

int main()
{
  try
  {

    udp::endpoint destination(boost::asio::ip::address::from_string("192.168.1.11"), 100);

    std::size_t sent_size;
    boost::asio::io_service io_service;
    boost::asio::ip::address_v4 stmIP;
    stmIP.from_string(ip_address);
    udp::socket mySocket(io_service, udp::v4());
    udp::endpoint remote_endpoint(stmIP, 100);

    boost::array<char, 350> send_buf;
    for(;;){
      sent_size = mySocket.send_to(boost::asio::buffer(send_buf), destination);
      std::cout << "spediti " << sent_size << std::endl;
      sleep(2);
    }


  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}