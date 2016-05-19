#include <iostream>
#include <boost/asio.hpp>
#include <string>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <vizzy_expressions/ExpressionAction.h>
#include "../include/expression_vals.hpp"

//TODO - Use exceptions to cleanly terminate if there is an error connecting to the board


using namespace boost::asio;



class DriverComms{

private:
io_service io;
boost::system::error_code ec;



public:
serial_port *sp;

DriverComms(std::string port, int baudrate){

    sp = new serial_port(io);
    //connect to port
    sp->open(port, ec);
    if(ec)
    {
        //Failure to connect
        ROS_ERROR_STREAM("Error connecting to expression board: " << ec.message());
        exit(-1);
    }

    //Set the baudrate
    sp->set_option(serial_port_base::baud_rate(baudrate));   

}

bool sendCommand(std::string value)
{
    int tries = 0;

    while(tries < 10)
    {
    int sentSize = write(*sp, boost::asio::buffer(value));

    if(sentSize == value.size())
      {
        return true;
      }
    else
      tries++;
    }

    return false;
}

~DriverComms()
{

    delete sp;
}

};

class ExpressionDriver{

private:
ros::NodeHandle nh_;
ros::NodeHandle nPriv;
DriverComms *driverComms;

std::string port;
int baudrate;

actionlib::SimpleActionServer<vizzy_expressions::ExpressionAction> as_;
std::string action_name_;

vizzy_expressions::ExpressionFeedback feedback_;
vizzy_expressions::ExpressionResult result_;


public:
ExpressionDriver(std::string name) : nPriv("~"), as_(nh_, name, boost::bind(&ExpressionDriver::executeCB, this, _1), false),
    action_name_()
{

    //Initialize the port
    nPriv.param<std::string>("port", port, "/dev/ttyACM0");
    nPriv.param<int>("baudrate", baudrate, 115200);
    driverComms = new DriverComms(port, baudrate);

    //Start the action server
    as_.start();

    //Initialize and set the default expression
    feedback_.mouth_emotion = vizzy_expressions::ExpressionActionGoal::_goal_type::FACE_HAPPY;
    feedback_.eyelids_emotion = vizzy_expressions::ExpressionActionGoal::_goal_type::FACE_HAPPY;
    feedback_.rightEyebrow_emotion = vizzy_expressions::ExpressionActionGoal::_goal_type::FACE_HAPPY;
    feedback_.leftEyebrow_emotion = vizzy_expressions::ExpressionActionGoal::_goal_type::FACE_HAPPY;

    driverComms->sendCommand(ExpressionValues::ExpressionHappy::RIGHT_EYEBROW);


//    feedback_.mouth_value =


}

void executeCB(const vizzy_expressions::ExpressionGoalConstPtr &goal)
{
    bool success = true;

    //This is kinda useless... I don't have feedback from the board :'(
//    feedback_.eyelids_emotion
}

~ExpressionDriver()
{
    delete driverComms;
}

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ExpressionDriver");

    ExpressionDriver expressionDriver(ros::this_node::getName());



    ros::spin();

    return 0;
}
