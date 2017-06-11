#ifndef CMD_ABS_H
#define CMD_ABS_H

#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <elikos_ros/TrajectoryCmd.h>

enum PriorityLevel {
        LANDING,
        OFFBOARD,
        INTERACTING,
        ALWAYS_ABORTABLE
    };

class CmdAbs
{
public:


    CmdAbs(ros::NodeHandle* nh, int id);
    virtual ~CmdAbs() = default;

    virtual void execute() = 0;
    virtual void abort() = 0;
    virtual void ajustement(geometry_msgs::Pose destination, trajectory_msgs::MultiDOFJointTrajectory trajectory);

    inline void setId(int id);
    inline int getId() const;
    inline void setTrajectory(trajectory_msgs::MultiDOFJointTrajectory cmdTraj);
    inline void setDestination(geometry_msgs::Pose cmdDest);
    inline uint8_t getCmdCode();
    inline PriorityLevel getCmdPriority();


    const std::string MAV_FRAME = { "elikos_fcu" };
    const std::string WORLD_FRAME = { "elikos_arena_origin" };

protected:
    int id_ = -1;
    trajectory_msgs::MultiDOFJointTrajectory cmdTrajectory_;
    geometry_msgs::Pose cmdDestination_;
    uint8_t cmdCode_;

    bool isAborted_;

    ros::NodeHandle* nh_;

    PriorityLevel cmdPriority_;

    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

private:
    CmdAbs() = delete;

};

inline int CmdAbs::getId() const
{
    return id_;
}

inline void CmdAbs::setId(int id)
{
    id_ = id;
}

inline void CmdAbs::setTrajectory(trajectory_msgs::MultiDOFJointTrajectory cmdTraj)
{
    cmdTrajectory_ = cmdTraj;
}

inline void CmdAbs::setDestination(geometry_msgs::Pose cmdDest)
{
    cmdDestination_ = cmdDest;
}

inline uint8_t CmdAbs::getCmdCode()
{
    return cmdCode_;
}

inline PriorityLevel CmdAbs::getCmdPriority()
{
    return cmdPriority_;
}

#endif /// CMD_ABS