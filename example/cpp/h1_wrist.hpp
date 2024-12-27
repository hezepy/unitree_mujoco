#pragma once

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
// DDS Channel
#include "include/dds/Publisher.h"
#include "include/dds/Subscription.h"
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

#include <eigen3/Eigen/Dense>
#include <unordered_map>

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

constexpr int kNumMotors = 20;
constexpr float kPi = 3.141592654;
constexpr float kPi_2 = 1.57079632;
constexpr float kPi_4 = 0.785398163;

/**
 * @brief Unitree H1 Wrist Controller
 * The user can subscribe to "rt/wrist/state" to get the current state of the wrist and publish to "rt/wrist/cmd" to control the hand.
 * 
 *                  IDL Types
 * user ---(unitree_go::msg::dds_::MotorCmds_)---> "rt/wrist/cmd"
 * user <--(unitree_go::msg::dds_::MotorStates_)-- "rt/wrist/state"
 */
class H1WristController
{
public:
    H1WristController()
    {
        this->InitDDS_();
    }

    // DDS parameters
    std::unique_ptr<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorCmds_>> wristcmd;
    std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>> wriststate;
private:
    void InitDDS_()
    {
        wristcmd = std::make_unique<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorCmds_>>(
            "rt/wrist/cmd");
        wristcmd->msg_.cmds().resize(2);
        wriststate = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>>(
            "rt/wrist/state");
        wriststate->msg_.states().resize(2);
    }
};