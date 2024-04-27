#pragma once

#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/MoveBaseGoal.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <mbf_msgs/MoveBaseResult.h>
#include </home/ros/catkin_ws/src/mbf_tutorials/mbf_advanced/include/mbf_advanced/helpers2.h>

namespace mbf_advanced
{

enum MBFCircleClientState
{
    AT_END = 0,
    MOVING,
    FAILED
};

struct MBFCircleClient
{
    explicit MBFCircleClient(std::vector<mbf_msgs::MoveBaseGoal> pose_goals)
            : pose_goals_(std::move(pose_goals))
            , it_(pose_goals_.begin())
            , prev_move_(pose_goals_.begin())
            , home_(pose_goals_.back())
            , ac_("move_base_flex/move_base", true)
            , terminate_(false)
            , goalsize(pose_goals_.size())
    {
        ac_.waitForServer();
        ROS_INFO("Connected to MBF action server");
    }



    auto move_to_goal()
    {
        auto& goal= pose_goals_[np];
        ROS_INFO_STREAM("Attempting to reach " << goal.target_pose.pose.position.x << " / " << goal.target_pose.pose.position.y);
        auto result = mbf_advanced::move(ac_, goal);
        np++;//change index of goal vector-->next goal
        if (result->outcome ==mbf_msgs::MoveBaseResult::SUCCESS)
            ns++; //count number of success
        if (np>pose_goals_.size())
            np=0;
        return result;
    }


    std::vector<mbf_msgs::MoveBaseGoal> pose_goals_;
    std::vector<mbf_msgs::MoveBaseGoal>::const_iterator it_;
    std::vector<mbf_msgs::MoveBaseGoal>::const_iterator prev_move_;
    const mbf_msgs::MoveBaseGoal& home_;
    actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> ac_;
    bool terminate_;
    int np=0,ns=0;
    int goalsize;
};

} // end
