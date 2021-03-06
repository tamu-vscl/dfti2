#ifndef AUTO_EXCITATION_H
#define AUTO_EXCITATION_H

#define SIGNAL_TYPE_IGNORE_CHANNEL 0
#define SIGNAL_TYPE_OFFSET 1
#define SIGNAL_TYPE_SQUARE 2
#define SIGNAL_TYPE_SAWTOOTH 3
#define SIGNAL_TYPE_SINE 4
#define SIGNAL_TYPE_SINE_SWEEP 5
#define SIGNAL_TYPE_THROTTLE_MAX_IDLE 6
#define SIGNAL_TYPE_DOUBLET 7
#define SIGNAL_TYPE_TRIPLET 8
#define SIGNAL_TYPE_T2O2 9

#define PI 3.14159
#define IGNORE_CHANNEL UINT16_MAX

#include <ros/ros.h>

#include <mavros_msgs/OverrideRCIn.h>
#include <std_srvs/Trigger.h>
#include <mavros_msgs/RCIn.h>

class AutoExcitation
{
public:

  struct Signal
  {
    // Signal parameters
    int type = SIGNAL_TYPE_IGNORE_CHANNEL; // Type of signal
    double period = 1; // Length of 1 period in seconds.
    double amplitude = 0; // Scaling factor for signal (exact implimentation varies between signal types) TODO: Units
    double offset = 1500; // Moves the signal up or down to center it around a value other then 0
    double f0 = 1.0; // Intial frequency for sine sweep
    double f1 = 10.0; // Final frequency for sine sweep
    double delay = 0; // Seconds before this signal begins

    // Logistical parameters
    double percent_of_period = 0; // Tracks where in the period the signal is curently
    double value = IGNORE_CHANNEL; // Actual ouput of the signal generator
    double length = 1; // Number of periods to execute. Can be partial period(s)
  };

  AutoExcitation();
  inline const Signal& signal(int i) const { return signal_[i]; }

private:

  ros::NodeHandle nh_;
  ros::Publisher offboard_override_pub_;
  ros::Subscriber rc_in_sub_;
  ros::ServiceServer arm_disturb_input_svc_;

  bool arm_auto_excitation_callback(std_srvs::Trigger::Request  &req,
                                    std_srvs::Trigger::Response &res);

  void receive_rc_in_callback(const mavros_msgs::RCIn &rc_in);
  void update_signals();
  void update_signal(int i);
  void load_param();

  Signal signal_ [8];
  double arm_time_out_time_;
  int trigger_channel_;
  double update_rate_;
  double signal_start_time_;

  unsigned int rc_trigger_channel_value;
};

#endif // AUTO_EXCITATION_H
