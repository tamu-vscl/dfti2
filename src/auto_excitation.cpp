/*
 * Copyright (c) 2019 Kameron Eves TAMU VSCL.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyrighhttps://github.com/tamu-vscl/dfti2.gitt holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software wit#include <ros/ros.h>hout specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
The purpose of this node is to inject some signal into the outputs of the
rosflight board. This is useful, among other things, for system identification
*/

#include <cmath>
#include <vector>

#include <ros/ros.h>

#include <dfti2/auto_excitation.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "auto_excitation");
  AutoExcitation auto_excitation;
  ROS_INFO("Auto excitation is ready!");
  ros::spin();
}

AutoExcitation::AutoExcitation()
{
  offboard_override_pub_ = nh_.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override",1000);

  load_param();

  arm_disturb_input_svc_ = nh_.advertiseService("arm_auto_excitation", &AutoExcitation::arm_auto_excitation_callback,this);
}

bool AutoExcitation::arm_auto_excitation_callback(std_srvs::Trigger::Request  &req,
                                                  std_srvs::Trigger::Response &res)
{
  res.success = false;

  load_param();

  ROS_WARN("Auto excitation armed!");
  double arm_time = ros::Time::now().toSec();
  ros::Rate loop_rate(10);
  boost::shared_ptr<mavros_msgs::RCIn const> rc_raw_msg;
  while (1)
  {
    rc_raw_msg = ros::topic::waitForMessage<mavros_msgs::RCIn>("/mavros/rc/in");
    if(rc_raw_msg->channels[trigger_channel_]>1900)
    {
      ROS_WARN("Initiating auto excitation!");
      break;
    }

    if ((ros::Time::now().toSec()-arm_time)>=arm_time_out_time_)
    {
      ROS_INFO("The auto excitation arm has timed out.");
      return res.success;
    }

    loop_rate.sleep();
  }

  loop_rate = ros::Rate(update_rate_);
  signal_start_time_ = ros::Time::now().toSec();
  mavros_msgs::OverrideRCIn msg;
  while (!res.success)
  {
    update_signals();

    for (int i = 0;i<8;i++)
    {
      msg.channels[i] = signal_[i].value;
    }

    offboard_override_pub_.publish(msg);
    // TODO: add multi input

    res.success = true;
    for(int i = 0;i<8;i++){
      if(signal_[i].percent_of_period < signal_[i].length)
      {
        res.success = false;
        break;
      }
    }

    // rc_raw_msg = ros::topic::waitForMessage<mavros_msgs::RCIn>("/mavros/rc/in");
    // if(rc_raw_msg->channels[trigger_channel_]<=1900)
    // {
    //   ROS_ERROR("Pilot Overrided the auto excitation!");
    //   break;
    // }

    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("Auto excitation complete.");

  return res.success;
}

void AutoExcitation::update_signals()
{
  for(int i = 0;i<8;i++)
  {
    if(signal_[i].type != IGNORE_CHANNEL)
    {
      signal_[i].percent_of_period = (ros::Time::now().toSec() - (signal_start_time_ + signal_[i].delay))/signal_[i].period;
    }
    if(0<=signal_[i].percent_of_period && signal_[i].percent_of_period<=signal_[i].length){update_signal(i);}
    else{signal_[i].value = IGNORE_CHANNEL;}
  }
}

void AutoExcitation::update_signal(int i)
{
  switch(signal_[i].type)
  {
  case SIGNAL_TYPE_OFFSET:
    signal_[i].value = signal_[i].offset;
    break;
  case SIGNAL_TYPE_SQUARE: // Note that the square wave and ramp offset is bassed off of the zero point. i.e. an offset of 0 will cause the wave to oscilate between 0 and 1 (if the amplitude is 1).
    signal_[i].value = floor(signal_[i].percent_of_period)>=0.5 ? signal_[i].amplitude+signal_[i].offset : signal_[i].offset;
    break;
  case SIGNAL_TYPE_SAWTOOTH: // Note that the square wave and ramp offset is bassed off of the zero point. i.e. an offset of 0 will cause the wave to oscilate between 0 and 1 (if the amplitude is 1).
    signal_[i].value = signal_[i].amplitude*floor(signal_[i].percent_of_period) + signal_[i].offset;
    break;
  case SIGNAL_TYPE_SINE: // Note that the sin wave offset is bassed off of the middle point. i.e. an offset of 0 will cause the wave to oscilate between -1 and 1 (if the amplitude is 1).
    signal_[i].value = signal_[i].amplitude*sin(signal_[i].percent_of_period*2*PI) + signal_[i].offset;
    break;
  case SIGNAL_TYPE_SINE_SWEEP:
    signal_[i].value = signal_[i].amplitude*sin(2*PI*((signal_[i].f1-signal_[i].f0)/2*pow(signal_[i].percent_of_period,2)*signal_[i].period + signal_[i].f0*signal_[i].percent_of_period*signal_[i].period)) + signal_[i].offset;
    break;
  default: // This tells the autopilot to ignore this signal and the RC signal is used instead.
    signal_[i].value = IGNORE_CHANNEL;
    break;
  }
}

void AutoExcitation::load_param()
{
  std::vector<int> type;
  std::vector<double> period;
  std::vector<double> amplitude;
  std::vector<double> offset;
  std::vector<double> f0;
  std::vector<double> f1;
  std::vector<double> delay;

  std::vector<double> length;

  // load parameters
  nh_.getParam("type",type);
  nh_.getParam("period",period);
  nh_.getParam("amplitude",amplitude);
  nh_.getParam("offset",offset);
  nh_.getParam("f0",f1);
  nh_.getParam("f1",f0);
  nh_.getParam("delay",delay);
  nh_.getParam("length",length);

  for(int i = 0;i<8;i++)
  {
    signal_[i].type = type[i];
    signal_[i].period = period[i];
    signal_[i].amplitude = amplitude[i];
    signal_[i].offset = offset[i];
    signal_[i].f0 = f0[i];
    signal_[i].f1 = f1[i];
    signal_[i].delay = delay[i];
    signal_[i].percent_of_period = 0;
    signal_[i].value = IGNORE_CHANNEL;
    signal_[i].length = length[i];
  }

  nh_.getParam("arm_time_out_time",arm_time_out_time_);
  nh_.getParam("trigger_channel",trigger_channel_);
  nh_.getParam("update_rate",update_rate_);
}