/**
 * @brief APM PID Tuning plugin
 * @file pid_tuning.cpp
 * @author Assistant
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2025.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "mavros_msgs/msg/pid_tuning.hpp"

namespace mavros
{
namespace extra_plugins
{
/**
 * @brief PID tuning plugin
 *
 * Handles receiving PID tuning data from FCU and sending tuning data to FCU.
 */
class PIDTuningPlugin : public plugin::Plugin
{
public:
    explicit PIDTuningPlugin(plugin::UASPtr uas_) : Plugin(uas_, "pid_tuning") {

        // Publisher for PID tuning data received from FCU
        pid_tuning_pub = node->create_publisher<mavros_msgs::msg::PidTuning>("~/pid_tuning", 10);
        
        // Subscriber for PID tuning data to be sent to FCU
        pid_tuning_sub = node->create_subscription<mavros_msgs::msg::PidTuning>(
            "~/pid_tuning_input", 10, 
            std::bind(&PIDTuningPlugin::pid_tuning_cb, this, std::placeholders::_1));
    }

    Subscriptions get_subscriptions() override
    {
        return {
            make_handler(&PIDTuningPlugin::handle_pid_tuning),
        };
    }

private:
    rclcpp::Publisher<mavros_msgs::msg::PidTuning>::SharedPtr pid_tuning_pub;
    rclcpp::Subscription<mavros_msgs::msg::PidTuning>::SharedPtr pid_tuning_sub;

    /* -*- low-level send -*- */
    /**
     * @brief Send PID tuning data to the FCU
     *
     * @param axis      Axis to tune
     * @param desired   Desired value
     * @param achieved  Achieved value
     * @param ff        Feedforward gain
     * @param p         Proportional gain
     * @param i         Integral gain
     * @param d         Derivative gain
     */
    void send_pid_tuning(const uint8_t axis, const float desired, const float achieved,
                        const float ff, const float p, const float i, const float d)
    {
        mavlink::ardupilotmega::msg::PID_TUNING pt {};

        pt.axis = axis;
        pt.desired = desired;
        pt.achieved = achieved;
        pt.FF = ff;
        pt.P = p;
        pt.I = i;
        pt.D = d;

        uas->send_message(pt);
    }

    /* -*- callbacks -*- */
    /**
     * @brief Handle incoming PID_TUNING messages from FCU
     */
    void handle_pid_tuning(const mavlink::mavlink_message_t *msg [[maybe_unused]], 
                           mavlink::ardupilotmega::msg::PID_TUNING &pid, 
                           plugin::filter::SystemAndOk filter [[maybe_unused]])
    {
        auto pid_tuning_msg = std::make_unique<mavros_msgs::msg::PidTuning>();

        pid_tuning_msg->header.stamp = node->now();
        pid_tuning_msg->header.frame_id = "base_link";
        
        pid_tuning_msg->axis = pid.axis;
        pid_tuning_msg->desired = pid.desired;
        pid_tuning_msg->achieved = pid.achieved;
        pid_tuning_msg->ff = pid.FF;
        pid_tuning_msg->p = pid.P;
        pid_tuning_msg->i = pid.I;
        pid_tuning_msg->d = pid.D;

        pid_tuning_pub->publish(std::move(pid_tuning_msg));
    }

    /**
     * @brief Callback for incoming PID tuning ROS messages to be sent to FCU
     */
    void pid_tuning_cb(const mavros_msgs::msg::PidTuning::SharedPtr req)
    {
        send_pid_tuning(
            req->axis,
            req->desired,
            req->achieved,
            req->ff,
            req->p,
            req->i,
            req->d
        );
    }
};
}   // namespace extra_plugins
}   // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::PIDTuningPlugin)
