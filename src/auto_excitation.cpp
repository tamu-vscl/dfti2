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
 * * Neither the name of the copyright holder nor the names of its
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

#include <ros/ros.h>

#include <dfti2/auto_excitation.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "auto_excitation");
  AutoExcitation auto_excitation;
  ros::spin();
}

AutoExcitation::AutoExcitation()
{
  offboard_override_pub_ = nh_.advertise<rosflight_msgs::AuxCommand>("/mavros/rc/override",1);

  load_param();

  arm_disturb_input_svc_ = nh_.advertiseService("arm_disturb_input", &AutoExcitation::arm_disturb_input_callback,this);
}

bool AutoExcitation::arm_disturb_input_callback(dfti2::DisturbInput::Request  &req,
                                                dfti2::DisturbInput::Response &res)
{
  res.finished = false;

  load_param();

  if(dir_svc.response.exists)
  {
    ROS_WARN("Channel %i disturbance armed!",req.channel);
    signal_.last_update = ros::Time::now().toSec();
    while (1)
    {
      boost::shared_ptr<mavros_msgs::RCIn const> rc_raw_msg = ros::topic::waitForMessage<mavros_msgs::RCIn>("/mavros/rc/in");
      if(rc_raw_msg->channels[trigger_channel_]>1900)
      {
        ROS_WARN("Initiating disturbance on channel %i",req.channel);
        break;
      }

      if ((ros::Time::now().toSec()-signal_.last_update)>=arm_time_out_time_)
      {
        ROS_INFO("The disturbance arm has timed out.");
        signal_ = Signal();
        return res.finished;
      }
    }
  }
  else
  {
    ROS_ERROR("Unable to arm disturbance. Check that the trigger channel is set correctly on transmitter.");
    signal_ = Signal();
    return res.finished;
  }

  ros::Rate loop_rate(signal_.update_rate);
  signal_.last_update = ros::Time::now().toSec();
  while (!res.finished)
  {
    update_signal();
    mavros_msgs::OverrideRCIn msg;
    for (int i = 0;i<8;i++)
    {
      msg.type_array[i] = UINT16_MAX;
      msg.values[i] = 0.0;
    }
    msg.type_array[req.channel] = output_type_;
    msg.values[req.channel] = signal_.value;
    offboard_override_pub_.publish(msg);
    // TODO: include servo vs motor adjustment
    // TODO: add multi input
    res.finished = signal_.length <= (signal_.period_count+signal_.percent_of_period);
    loop_rate.sleep();
  }

  signal_ = Signal();

  return res.finished;
}

void AutoExcitation::update_signal()
{
  double current_time = ros::Time::now().toSec();
  float step = current_time - signal_.last_update;
  signal_.percent_of_period = step/signal_.period;
  if (floor(signal_.percent_of_period)!=0)
  {
    signal_.percent_of_period-=floor(signal_.percent_of_period);
    signal_.period_count++;
    signal_.last_update = current_time;
  }

  switch(signal_.type)
  {
  case SIGNAL_TYPE_OFFSET:
    signal_.value = signal_.offset;
    break;
  case SIGNAL_TYPE_SQUARE: // Note that the square wave and ramp offset is bassed off of the zero point. i.e. an offset of 0 will cause the wave to oscilate between 0 and 1 (if the amplitude is 1).
    signal_.value = signal_.percent_of_period>=0.5 ? signal_.amplitude+signal_.offset : signal_.offset;
    break;
  case SIGNAL_TYPE_RAMP: // Note that the square wave and ramp offset is bassed off of the zero point. i.e. an offset of 0 will cause the wave to oscilate between 0 and 1 (if the amplitude is 1).
    signal_.value = signal_.amplitude*signal_.percent_of_period + signal_.offset;
    break;
  case SIGNAL_TYPE_SINE: // Note that the sin wave offset is bassed off of the middle point. i.e. an offset of 0 will cause the wave to oscilate between -1 and 1 (if the amplitude is 1).
    signal_.value = signal_.amplitude*sin(signal_.percent_of_period*2*PI) + signal_.offset;
    break;
  case SIGNA_TYPE_SINE_SWEEP:
    signal_.value = signal_.amplitude*sin(2*PI*signal_.f0*signal_.period*(pow(signal_.f1/signal_.f0,signal_.percent_of_period)-1)/(log(signal_.f1)-log(signal_.f0)) + signal_.offset;
    break;
  default: // This tells the autopilot to ignore this signal and the RC signal is used instead.
    signal_.value = UINT16_MAX;
    break;
  }
}

void AutoExcitation::load_param()
{
  // load parameters
  nh_.param("type",signal_.type,0);
  if(nh_.hasParam("period"))
  {
    nh_.param("period",signal_.period,1.0);
  }
  else
  {
    float frequency;
    nh_.getParam("f0",frequency);
    signal_.period = 1.0/frequency;
  }
  nh_.param("amplitude",signal_.amplitude,0.0);
  nh_.param("offset",signal_.offset,0.0);
  nh_.param("length",signal_.length,1.0);
  nh_.param("settling_time",signal_.settling_time,5.0);
  nh_.param("arm_time_out_time",arm_time_out_time_,10.0);
  nh_.param("update_rate",signal_.update_rate,100.0);
  nh_.param("f0",signal_.f1,1.0);
  nh_.param("f1",signal_.f0,10.0);
  nh_.param("output_type",output_type_,2);
  nh_.param("trigger_channel",trigger_channel_,9);
}
