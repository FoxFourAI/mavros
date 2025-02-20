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

#include <mavros/mavros_plugin.h>
#include <mavros_msgs/PidTuning.h>

namespace mavros
{
namespace extra_plugins
{
/**
 * @brief PID tuning plugin
 *
 * Handles receiving PID tuning data from FCU and sending tuning data to FCU.
 */
class PIDTuningPlugin : public plugin::PluginBase
{
public:
    PIDTuningPlugin() : PluginBase(),
        sp_nh("~pid_tuning")
    { }

    void initialize(UAS &uas_) override
    {
        PluginBase::initialize(uas_);

        // Publisher for PID tuning data received from FCU
        pid_tuning_pub = sp_nh.advertise<mavros_msgs::PidTuning>("pid_tuning", 10);
        
        // Subscriber for PID tuning data to be sent to FCU
        pid_tuning_sub = sp_nh.subscribe("pid_tuning_input", 10, &PIDTuningPlugin::pid_tuning_cb, this);
    }

    Subscriptions get_subscriptions() override
    {
        return {
            make_handler(&PIDTuningPlugin::handle_pid_tuning),
        };
    }

private:
    ros::NodeHandle sp_nh;
    ros::Publisher pid_tuning_pub;
    ros::Subscriber pid_tuning_sub;

    /* -*- low-level send -*- */
    /**
     * @brief Send PID tuning data to the FCU
     *
     * @param axis      Axis to tune
     * @param desired   Desired value
     * @param achieved  Achieved value
     * @param FF        Feedforward gain
     * @param P         Proportional gain
     * @param I         Integral gain
     * @param D         Derivative gain
     */
    void send_pid_tuning(const uint8_t axis, const float desired, const float achieved,
                        const float FF, const float P, const float I, const float D)
    {
        mavlink::ardupilotmega::msg::PID_TUNING pt {};

        pt.axis = axis;
        pt.desired = desired;
        pt.achieved = achieved;
        pt.FF = FF;
        pt.P = P;
        pt.I = I;
        pt.D = D;

        UAS_FCU(m_uas)->send_message_ignore_drop(pt);
    }

    /* -*- callbacks -*- */
    /**
     * @brief Handle incoming PID_TUNING messages from FCU
     */
    void handle_pid_tuning(const mavlink::mavlink_message_t *msg, mavlink::ardupilotmega::msg::PID_TUNING &pid)
    {
        auto pid_tuning_msg = boost::make_shared<mavros_msgs::PidTuning>();

        pid_tuning_msg->header.stamp = ros::Time::now();
        pid_tuning_msg->header.frame_id = "base_link";
        
        pid_tuning_msg->axis = pid.axis;
        pid_tuning_msg->desired = pid.desired;
        pid_tuning_msg->achieved = pid.achieved;
        pid_tuning_msg->FF = pid.FF;
        pid_tuning_msg->P = pid.P;
        pid_tuning_msg->I = pid.I;
        pid_tuning_msg->D = pid.D;

        pid_tuning_pub.publish(pid_tuning_msg);
    }

    /**
     * @brief Callback for incoming PID tuning ROS messages to be sent to FCU
     */
    void pid_tuning_cb(const mavros_msgs::PidTuning::ConstPtr &req)
    {
        send_pid_tuning(
            req->axis,
            req->desired,
            req->achieved,
            req->FF,
            req->P,
            req->I,
            req->D
        );
    }
};
}   // namespace extra_plugins
}   // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::PIDTuningPlugin, mavros::plugin::PluginBase)
