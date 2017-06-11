#include <string>

#include <ros/ros.h>
#include "CmdOffBoard.h"
#include "CmdLanding.h"
#include "CmdFrontInteraction.h"
#include "CmdTopInteraction.h"
#include "CmdTravel.h"
#include "CmdStandBy.h"

#include "CmdExecutor.h"

CmdExecutor::CmdExecutor()
{

    commands_.insert(std::pair<int, std::unique_ptr<CmdOffBoard>>(0, std::unique_ptr<CmdOffBoard>(new CmdOffBoard(&nh_, -1))));
    commands_.insert(std::pair<int, std::unique_ptr<CmdLanding>>(1, std::unique_ptr<CmdLanding>(new CmdLanding(&nh_, -1))));
    commands_.insert(std::pair<int, std::unique_ptr<CmdFrontInteraction>>(2, std::unique_ptr<CmdFrontInteraction>(new CmdFrontInteraction(&nh_, -1))));
    commands_.insert(std::pair<int, std::unique_ptr<CmdTopInteraction>>(3, std::unique_ptr<CmdTopInteraction>(new CmdTopInteraction(&nh_, -1))));
    commands_.insert(std::pair<int, std::unique_ptr<CmdTravel>>(4, std::unique_ptr<CmdTravel>(new CmdTravel(&nh_, -1))));
    commands_.insert(std::pair<int, std::unique_ptr<CmdStandBy>>(5, std::unique_ptr<CmdStandBy>(new CmdStandBy(&nh_, -1))));
    currentCmd_ = commands_[5].get();
    pendingCmd_ = currentCmd_;

    cmdExecutionThread_ = std::thread(&CmdExecutor::executeCurrentCmd, this);
}

CmdExecutor::~CmdExecutor() 
{

}


OrderToGive CmdExecutor::checkNextOrder()
{
    if (currentCmd_->getCmdPriority() < pendingCmd_->getCmdPriority())
        return OrderToGive::CONTINUE;
    else if(currentCmd_->getCmdPriority() == pendingCmd_->getCmdPriority())
        if  (
                currentCmd_->getCmdCode() == pendingCmd_->getCmdCode() &&
                (currentCmd_->getCmdCode() == 2 || currentCmd_->getCmdCode() == 3 || currentCmd_->getCmdCode() == 4)
            )
            return OrderToGive::AJUST;
        else
            return OrderToGive::ABORT;
    else
        return OrderToGive::ABORT;
}

void CmdExecutor::commandReceived(const CmdConfig& newCommand)
{
    pendingCmdLock_.lock();
    createCommand(newCommand);
    if (OrderToGive::ABORT == checkNextOrder())
        currentCmd_->abort();
    else if(OrderToGive::AJUST == checkNextOrder())
    {
        if (currentCmd_->getCmdCode() == 2 || currentCmd_->getCmdCode() == 3 || currentCmd_->getCmdCode() == 4) // Robot interaction || travel
        {
            currentCmd_->ajustement(pendingDestination_, pendingTrajectory_);
        }
    }
    pendingCmdLock_.unlock();
}

void CmdExecutor::createCommand(const CmdConfig& config)
{

    std::unordered_map<int, std::unique_ptr<CmdAbs>>::iterator it;
    it = commands_.find(config.cmdCode_);
    if (it != commands_.end())
    {
        pendingCmd_ = it->second.get();
        it->second->setId(config.id_);
        if(it->first == 2 || it->first == 3) // Robot interaction
        {
            pendingDestination_ = config.cmdDestination_;
        } 
        if(it->first == 4) // Travel
        {
            pendingTrajectory_ = config.cmdTrajectory_;
        }
    }
}  

void CmdExecutor::executeCurrentCmd()
{
    while (true) 
    {
        currentCmd_->execute();
        pendingCmdLock_.lock();
        if(currentCmd_->getCmdCode() == 2 && static_cast<CmdFrontInteraction*>(currentCmd_)->getStatus() == CmdFrontInteraction::InteractionState::ASKS_FOR_OFFBOARD)
            currentCmd_ = commands_[0].get();
        else {
            currentCmd_ = pendingCmd_;
        }

        pendingCmd_ = commands_[5].get();
        pendingCmdLock_.unlock();
    }
}

void CmdExecutor::run()
{
    ros::Rate r(30);
    ros::spin();
}