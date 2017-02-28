#ifndef CMD_OFF_BOARD_H
#define CMD_OFF_BOARD_H

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "CmdAbs.h"

class CmdOffBoard : public CmdAbs
{
public:
    CmdOffBoard(ros::NodeHandle* nh);
    virtual ~CmdOffBoard() = default;

    virtual void execute();
    virtual void abort();
    virtual void ajustement();

    void stateCallBack(const mavros_msgs::State::ConstPtr& msg);

private:
    CmdOffBoard() = delete;

    ros::ServiceClient armingClient_;
    ros::ServiceClient setModeClient_;

    ros::Subscriber stateSub_;

    mavros_msgs::State currentState_;
    mavros_msgs::SetMode offbSetMode_;
    mavros_msgs::CommandBool armCmd_;

    ros::Time lastRequest_;

    
};

#endif /// CMD_OFF_BOARD_H
