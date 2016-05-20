#include <iostream>
#include <boost/asio.hpp>
#include <string>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <vizzy_expressions/ExpressionAction.h>
#include "../include/expression_vals.hpp"


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
            sp->close();
            delete sp;
            throw std::exception();
        }

        //Set the baudrate
        sp->set_option(serial_port_base::baud_rate(baudrate));

    }

    bool simpleSend(std::string msg)
    {
        int tries = 0;
        while(tries < 10)
        {
            int sentSize = write(*sp, boost::asio::buffer(msg));

            if(sentSize == (int) msg.size())
            {
                return true;
            }
            else
                tries++;
        }

        return false;
    }

    bool sendCommand(uint8_t value, std::string part)
    {

        std::stringstream msg;

        if(vizzy_expressions::ExpressionActionGoal::_goal_type::PART_RIGHTEYEBROW == part)
            msg << "R" << std::hex << std::uppercase << value;
        else if(vizzy_expressions::ExpressionActionGoal::_goal_type::PART_LEFTEYEBROW == part)
            msg << "L" << std::hex << std::uppercase << value;
        else if(vizzy_expressions::ExpressionActionGoal::_goal_type::PART_MOUTH == part)
            msg << "M" << std::hex << std::uppercase << value;
        else if(vizzy_expressions::ExpressionActionGoal::_goal_type::PART_EYELIDS == part)
            msg << "S" << std::hex << std::uppercase << value;
        else{
            ROS_ERROR("Invalid command!");
            return false;}


        return simpleSend(msg.str());

    }

    ~DriverComms()
    {
        //Disconnect from port
        sp->close();
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

        try
        {
            driverComms = new DriverComms(port, baudrate);
        }
        catch(const std::exception&)
        {
            throw std::exception();
        }

        //Start the action server
        as_.start();

        //Initialize and set the default expression
        bool sentM, sentS, sentR, sentL;

        sentM = driverComms->sendCommand(ExpressionValues::ExpressionHappy::MOUTH, vizzy_expressions::ExpressionActionGoal::_goal_type::PART_MOUTH);
        feedback_.mouth_value = ExpressionValues::ExpressionHappy::MOUTH;
        feedback_.mouth_emotion = vizzy_expressions::ExpressionActionGoal::_goal_type::FACE_HAPPY;

        sentS = driverComms->sendCommand(ExpressionValues::ExpressionHappy::EYELIDS, vizzy_expressions::ExpressionActionGoal::_goal_type::PART_EYELIDS);
        feedback_.eyelids_emotion = vizzy_expressions::ExpressionActionGoal::_goal_type::FACE_HAPPY;
        feedback_.eyelids_value = ExpressionValues::ExpressionHappy::EYELIDS;

        sentR = driverComms->sendCommand(ExpressionValues::ExpressionHappy::RIGHT_EYEBROW, vizzy_expressions::ExpressionActionGoal::_goal_type::PART_RIGHTEYEBROW);
        feedback_.rightEyebrow_emotion = vizzy_expressions::ExpressionActionGoal::_goal_type::FACE_HAPPY;
        feedback_.rightEyebrow_value = ExpressionValues::ExpressionHappy::RIGHT_EYEBROW;

        sentL = driverComms->sendCommand(ExpressionValues::ExpressionHappy::LEFT_EYEBROW, vizzy_expressions::ExpressionActionGoal::_goal_type::PART_LEFTEYEBROW);
        feedback_.leftEyebrow_emotion = vizzy_expressions::ExpressionActionGoal::_goal_type::FACE_HAPPY;
        feedback_.leftEyebrow_value = ExpressionValues::ExpressionHappy::LEFT_EYEBROW;

        if(sentM && sentS && sentR && sentL)
        {
            ROS_INFO("Initialization commands sent. Vizzy is Happy! :D");
        }
        else
        {
            ROS_ERROR("Expression initialization failed. Check the board? :(");
            as_.shutdown();
            delete driverComms;
            throw std::exception();
        }

    }

    void executeCB(const vizzy_expressions::ExpressionGoalConstPtr &goal)
    {
        bool success = true;

        feedback_.state_reached = false;

        if(goal->mode == goal->PREDEFINED)
        {
            ROS_INFO("Changing emotion for part %s to %s", goal->subsystem.c_str(), goal->emotion.c_str());
            if(goal->subsystem == goal->PART_ALL)
            {
                bool sentM(false), sentS(false), sentR(false), sentL(false);
                //If the goal is a valid emotion
                if(goal->emotion == goal->FACE_ANGRY || goal->emotion == goal->FACE_CUNNING || goal->emotion == goal->FACE_EVIL
                        || goal->emotion == goal->FACE_HAPPY || goal->emotion == goal->FACE_NEUTRAL || goal->emotion == goal->FACE_SAD
                        || goal->emotion == goal->FACE_SHY || goal->emotion == goal->FACE_SURPRISED
                        )
                {

                    sentS = driverComms->sendCommand(ExpressionValues::ExpressionValue(goal->emotion, goal->PART_EYELIDS), goal->PART_EYELIDS);
                    if(sentS)
                    {
                        feedback_.eyelids_value = ExpressionValues::ExpressionValue(goal->emotion, goal->PART_EYELIDS);
                        feedback_.eyelids_emotion = goal->emotion;
                    }

                    as_.publishFeedback(feedback_);

                    sentL = driverComms->sendCommand(ExpressionValues::ExpressionValue(goal->emotion, goal->PART_LEFTEYEBROW), goal->PART_LEFTEYEBROW);
                    if(sentL)
                    {
                        feedback_.leftEyebrow_value = ExpressionValues::ExpressionValue(goal->emotion, goal->PART_LEFTEYEBROW);
                        feedback_.leftEyebrow_emotion = goal->emotion;
                    }

                    as_.publishFeedback(feedback_);

                    sentM = driverComms->sendCommand(ExpressionValues::ExpressionValue(goal->emotion, goal->PART_MOUTH), goal->PART_MOUTH);

                    if(sentM)
                    {
                        feedback_.mouth_value = ExpressionValues::ExpressionValue(goal->emotion, goal->PART_MOUTH);
                        feedback_.mouth_emotion = goal->emotion;
                    }

                    as_.publishFeedback(feedback_);

                    sentR = driverComms->sendCommand(ExpressionValues::ExpressionValue(goal->emotion, goal->PART_RIGHTEYEBROW), goal->PART_RIGHTEYEBROW);

                    if(sentR)
                    {
                        feedback_.rightEyebrow_value = ExpressionValues::ExpressionValue(goal->emotion, goal->PART_RIGHTEYEBROW);
                        feedback_.rightEyebrow_emotion = goal->emotion;
                    }

                    as_.publishFeedback(feedback_);



                }
                {
                    ROS_ERROR("Invalid expression...");
                    success = false;
                }

                success = sentS && sentL && sentM && sentR;

                feedback_.state_reached = success;
                as_.publishFeedback(feedback_);

            }else if(goal->subsystem == goal->PART_EYELIDS || goal->subsystem == goal->PART_LEFTEYEBROW || goal->subsystem == goal->PART_MOUTH || goal->subsystem == goal->PART_RIGHTEYEBROW)
            {
                success = driverComms->sendCommand(ExpressionValues::ExpressionValue(goal->emotion, goal->subsystem), goal->subsystem);

                if(success)
                {
                    if(goal->subsystem == goal->PART_EYELIDS)
                    {
                        feedback_.eyelids_value = ExpressionValues::ExpressionValue(goal->emotion, goal->PART_EYELIDS);
                        feedback_.eyelids_emotion = goal->emotion;
                    }
                    else if(goal->subsystem == goal->PART_LEFTEYEBROW)
                    {
                        feedback_.leftEyebrow_value = ExpressionValues::ExpressionValue(goal->emotion, goal->PART_LEFTEYEBROW);
                        feedback_.leftEyebrow_emotion = goal->emotion;

                    }else if(goal->subsystem == goal->PART_MOUTH)
                    {
                        feedback_.mouth_value = ExpressionValues::ExpressionValue(goal->emotion, goal->PART_MOUTH);
                        feedback_.mouth_emotion = goal->emotion;

                    }else if(goal->subsystem == goal->PART_RIGHTEYEBROW)
                    {
                        feedback_.rightEyebrow_value = ExpressionValues::ExpressionValue(goal->emotion, goal->PART_RIGHTEYEBROW);
                        feedback_.rightEyebrow_emotion = goal->emotion;
                    }
                }

                feedback_.state_reached = success;

                as_.publishFeedback(feedback_);


            }
            else
            {
                ROS_ERROR("Invalid part...");
                success = false;
            }

            result_.eyelids_emotion = feedback_.eyelids_emotion;
            result_.leftEyebrow_emotion = feedback_.leftEyebrow_emotion;
            result_.rightEyebrow_emotion = feedback_.rightEyebrow_emotion;
            result_.mouth_emotion = feedback_.mouth_emotion;

            result_.eyelids_value = feedback_.eyelids_value;
            result_.leftEyebrow_value = feedback_.leftEyebrow_value;
            result_.rightEyebrow_value = feedback_.rightEyebrow_value;
            result_.mouth_value = feedback_.mouth_value;

            result_.state_reached = success;


            if(success)
            {
                ROS_INFO("Emotion goal reached");
                as_.setSucceeded(result_);

            }
            else
            {
                ROS_INFO("Emotion goal failed...");
                as_.setAborted(result_);
            }

        }else if(goal->mode == goal-> LOWLEVEL)
        {

            std::stringstream m;
            std::stringstream s;
            std::stringstream r;
            std::stringstream l;


            m << std::hex << goal->mouth_value;
            s << std::hex << goal->eyelids_value;
            r << std::hex << goal->rightEyebrow_value;
            l << std::hex << goal->leftEyebrow_value;

            ROS_INFO("Changing to non predefined emotion. Mouth: %s, Eyelids: %s, RightEyebrow: %s, LeftEyebrow: %s", m.str().c_str(), s.str().c_str(), r.str().c_str(), l.str().c_str());
            bool sentM(false), sentS(false), sentR(false), sentL(false);

            sentM = driverComms->sendCommand(goal->mouth_value, goal->PART_MOUTH);
            sentS = driverComms->sendCommand(goal->eyelids_value, goal->PART_EYELIDS);
            sentR = driverComms->sendCommand(goal->rightEyebrow_value, goal->PART_RIGHTEYEBROW);
            sentL = driverComms->sendCommand(goal->leftEyebrow_value, goal->PART_LEFTEYEBROW);

            success = sentM && sentS && sentR && sentL;

            result_.eyelids_emotion = feedback_.eyelids_emotion = feedback_.FACE_UNDEFINED;
            result_.leftEyebrow_emotion = feedback_.leftEyebrow_emotion = feedback_.FACE_UNDEFINED;
            result_.rightEyebrow_emotion = feedback_.rightEyebrow_emotion = feedback_.FACE_UNDEFINED;
            result_.mouth_emotion = feedback_.mouth_emotion = feedback_.FACE_UNDEFINED;

            result_.eyelids_value = feedback_.eyelids_value;
            result_.leftEyebrow_value = feedback_.leftEyebrow_value;
            result_.rightEyebrow_value = feedback_.rightEyebrow_value;
            result_.mouth_value = feedback_.mouth_value;

            result_.state_reached = success;

            if(success)
            {
                ROS_INFO("Lowlevel emotion goal reached");
                as_.setSucceeded(result_);
            }
            else{
                ROS_INFO("Lowlevel emotion goal failed...");
                as_.setAborted(result_);
            }

        }
        else
        {
            ROS_ERROR("Invalid emotion mode...");
            result_.state_reached = false;
            as_.setAborted(result_);
        }
    }

    ~ExpressionDriver()
    {
        delete driverComms;
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ExpressionDriver");

    vizzy_expressions::ExpressionGoal goal;

    goal.mouth_value = 0x1A;

    // +a <- converts "a" automatically from char to int
    ROS_ERROR_STREAM("String: " << std::hex << std::uppercase << +goal.mouth_value);

    try
    {
        ExpressionDriver expressionDriver(ros::this_node::getName());
    }
    catch(const std::exception&)
    {
        ros::shutdown();
        return EXIT_FAILURE;
    }

    ros::spin();

    return 0;
}
