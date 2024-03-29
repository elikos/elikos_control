#include "CmdStandBy.h"

CmdStandBy::CmdStandBy(ros::NodeHandle* nh, int id)
    : CmdAbs(nh, id)
{
    cmdPriority_ = PriorityLevel::ALWAYS_ABORTABLE;
    cmdCode_ = elikos_msgs::DMCmd::STANDBY;
}

CmdStandBy::~CmdStandBy()
{
}

void CmdStandBy::execute()
{
    std_msgs::String msg;
    msg.data = "Stand by";
    statePubCommand_.publish(msg);

    isAborted_ = false;
    // TODO: Essayer a nouveau si le lookup echoue.
    try {
        tf_listener_.lookupTransform(WORLD_FRAME, MAV_FRAME, ros::Time(0), currentPosition_);
    } catch(tf::TransformException e) {
        ROS_ERROR("%s",e.what());
    }

    ros::Rate rate(30.0);
    while(ros::ok() && !isAborted_)
    {
        tf_broadcaster_.sendTransform(tf::StampedTransform(currentPosition_, ros::Time::now(), WORLD_FRAME, SETPOINT));
        ros::spinOnce();
        rate.sleep();
    }
}

void CmdStandBy::abort()
{
    isAborted_ = true;
}
