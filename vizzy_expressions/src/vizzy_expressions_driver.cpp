#include <iostream>
#include <boost/asio.hpp>
#include <string>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <vizzy_expressions/ExpressionAction.h>

using namespace boost::asio;



class DriverComms{

private:
io_service io;
boost::system::error_code ec;
serial_port *sp;


public:
DriverComms(std::string port, int baudrate){

    sp = new serial_port(io);

    //connect to port
    sp->open(port, ec);

    if(ec)
    {
        //Failure to connect
        ROS_ERROR_STREAM("Error connecting to expression board: " << ec.message());
    }

    //Set the baudrate
    sp->set_option(serial_port_base::baud_rate(baudrate));

}

~DriverComms()
{

    delete sp;
}

};

class ExpressionDriver{

private:
ros::NodeHandle nPriv;
DriverComms *driverComms;

std::string port;
int baudrate;

public:
ExpressionDriver() : nPriv("~"){

    nPriv.param<std::string>("port", port, "/dev/ttyACM0");
    nPriv.param<int>("baudrate", baudrate, 115200);





}

~ExpressionDriver()
{

}

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ExpressionDriver");

    ExpressionDriver expressionDriver();

    ros::spin();

}
