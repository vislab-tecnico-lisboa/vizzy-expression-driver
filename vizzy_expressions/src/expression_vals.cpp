#include "../include/expression_vals.hpp"
#include <ros/ros.h>
#include <vizzy_expressions/ExpressionAction.h>

using namespace ExpressionValues;

typedef vizzy_expressions::ExpressionActionGoal::_goal_type emo;

const uint8_t ExpressionNeutral::LEFT_EYEBROW = 0x02;
const uint8_t ExpressionNeutral::RIGHT_EYEBROW = 0x02;
const uint8_t ExpressionNeutral::MOUTH = 0x08;
const uint8_t ExpressionNeutral::EYELIDS = 0x38;

const uint8_t ExpressionHappy::LEFT_EYEBROW = 0x02;
const uint8_t ExpressionHappy::RIGHT_EYEBROW = 0x02;
const uint8_t ExpressionHappy::MOUTH = 0x0B;
const uint8_t ExpressionHappy::EYELIDS = 0x64;

const uint8_t ExpressionSad::LEFT_EYEBROW = 0x02;
const uint8_t ExpressionSad::RIGHT_EYEBROW = 0x02;
const uint8_t ExpressionSad::MOUTH = 0x38;
const uint8_t ExpressionSad::EYELIDS = 0x38;

const uint8_t ExpressionSurprised::LEFT_EYEBROW = 0x08;
const uint8_t ExpressionSurprised::RIGHT_EYEBROW = 0x08;
const uint8_t ExpressionSurprised::MOUTH = 0x16;
const uint8_t ExpressionSurprised::EYELIDS = 0x38;

const uint8_t ExpressionAngry::LEFT_EYEBROW = 0x01;
const uint8_t ExpressionAngry::RIGHT_EYEBROW = 0x01;
const uint8_t ExpressionAngry::MOUTH = 0x38;
const uint8_t ExpressionAngry::EYELIDS = 0x64;

const uint8_t ExpressionEvil::LEFT_EYEBROW = 0x01;
const uint8_t ExpressionEvil::RIGHT_EYEBROW = 0x01;
const uint8_t ExpressionEvil::MOUTH = 0x0B;
const uint8_t ExpressionEvil::EYELIDS = 0x64;

const uint8_t ExpressionShy::LEFT_EYEBROW = 0x04;
const uint8_t ExpressionShy::RIGHT_EYEBROW = 0x04;
const uint8_t ExpressionShy::MOUTH = 0x38;
const uint8_t ExpressionShy::EYELIDS = 0x64;

const uint8_t ExpressionCunning::LEFT_EYEBROW = 0x04;
const uint8_t ExpressionCunning::RIGHT_EYEBROW = 0x04;
const uint8_t ExpressionCunning::MOUTH = 0x38;
const uint8_t ExpressionCunning::EYELIDS = 0x38;

uint8_t ExpressionValues::ExpressionValue(std::string emotion, std::string part)
{

    if(emotion == emo::FACE_ANGRY)
    {
        if(part == emo::PART_EYELIDS)
          return ExpressionAngry::EYELIDS;
        else if(part == emo::PART_LEFTEYEBROW)
           return ExpressionAngry::LEFT_EYEBROW;
        else if(part == emo::PART_MOUTH)
           return ExpressionAngry::MOUTH;
        else if(part == emo::PART_RIGHTEYEBROW)
           return ExpressionAngry::RIGHT_EYEBROW;

    }else if(emotion == emo::FACE_CUNNING)
    {
        if(part == emo::PART_EYELIDS)
          return ExpressionCunning::EYELIDS;
        else if(part == emo::PART_LEFTEYEBROW)
           return ExpressionCunning::LEFT_EYEBROW;
        else if(part == emo::PART_MOUTH)
           return ExpressionCunning::MOUTH;
        else if(part == emo::PART_RIGHTEYEBROW)
           return ExpressionCunning::RIGHT_EYEBROW;

    }else if(emotion == emo::FACE_EVIL)
    {
        if(part == emo::PART_EYELIDS)
          return ExpressionEvil::EYELIDS;
        else if(part == emo::PART_LEFTEYEBROW)
           return ExpressionEvil::LEFT_EYEBROW;
        else if(part == emo::PART_MOUTH)
           return ExpressionEvil::MOUTH;
        else if(part == emo::PART_RIGHTEYEBROW)
           return ExpressionEvil::RIGHT_EYEBROW;

    }else if(emotion == emo::FACE_HAPPY)
    {
        if(part == emo::PART_EYELIDS)
          return ExpressionHappy::EYELIDS;
        else if(part == emo::PART_LEFTEYEBROW)
           return ExpressionHappy::LEFT_EYEBROW;
        else if(part == emo::PART_MOUTH)
           return ExpressionHappy::MOUTH;
        else if(part == emo::PART_RIGHTEYEBROW)
           return ExpressionHappy::RIGHT_EYEBROW;

    }else if(emotion == emo::FACE_NEUTRAL)
    {
        if(part == emo::PART_EYELIDS)
          return ExpressionNeutral::EYELIDS;
        else if(part == emo::PART_LEFTEYEBROW)
           return ExpressionNeutral::LEFT_EYEBROW;
        else if(part == emo::PART_MOUTH)
           return ExpressionNeutral::MOUTH;
        else if(part == emo::PART_RIGHTEYEBROW)
           return ExpressionNeutral::RIGHT_EYEBROW;

    }else if(emotion == emo::FACE_SAD)
    {
        if(part == emo::PART_EYELIDS)
          return ExpressionSad::EYELIDS;
        else if(part == emo::PART_LEFTEYEBROW)
           return ExpressionSad::LEFT_EYEBROW;
        else if(part == emo::PART_MOUTH)
           return ExpressionSad::MOUTH;
        else if(part == emo::PART_RIGHTEYEBROW)
           return ExpressionSad::RIGHT_EYEBROW;

    }else if(emotion == emo::FACE_SHY)
    {
        if(part == emo::PART_EYELIDS)
          return ExpressionShy::EYELIDS;
        else if(part == emo::PART_LEFTEYEBROW)
           return ExpressionShy::LEFT_EYEBROW;
        else if(part == emo::PART_MOUTH)
           return ExpressionShy::MOUTH;
        else if(part == emo::PART_RIGHTEYEBROW)
           return ExpressionShy::RIGHT_EYEBROW;

    }else if(emotion == emo::FACE_SURPRISED)
    {
        if(part == emo::PART_EYELIDS)
          return ExpressionSurprised::EYELIDS;
        else if(part == emo::PART_LEFTEYEBROW)
           return ExpressionSurprised::LEFT_EYEBROW;
        else if(part == emo::PART_MOUTH)
           return ExpressionSurprised::MOUTH;
        else if(part == emo::PART_RIGHTEYEBROW)
           return ExpressionSurprised::RIGHT_EYEBROW;

    }else
    {
        ROS_ERROR("Invalid expression...");
    }

    return 0x00;
}
