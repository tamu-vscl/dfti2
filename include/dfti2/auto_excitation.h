#ifndef AUTO_EXCITATION_H
#define AUTO_EXCITATION_H

#define SIGNAL_TYPE_OFFSET 0
#define SIGNAL_TYPE_SQUARE 1
#define SIGNAL_TYPE_RAMP 2
#define SIGNAL_TYPE_SINE 3
#define SIGNAL_TYPE_SINE_SWEEP 4

#define PI 3.14159

#include <ros/ros.h>

#include <mavros_msgs/OverrideRCIn.h>
#include <dfti2/DisturbInput.h>
#include <mavros_msgs/RCIn.h>

class AutoExcitation
{
public:

  struct Signal
  {
    int type = SIGNAL_TYPE_OFFSET;
    double period = 1;
    double amplitude = 0;
    double offset = 0;
    double last_update = -1;
    double percent_of_period = 0;
    double period_count = 0;
    double length = 1;
    double value = 0;
    double settling_time = 5;
    double update_rate = 100;
    double f0 = 1.0;
    double f1 = 10.0;
  };

  AutoExcitation();
  inline const Signal& signal() const { return signal_; }

private:

  ros::NodeHandle nh_;
  ros::Publisher offboard_override_pub_;
  ros::ServiceServer arm_disturb_input_svc_;

  bool arm_disturb_input_callback(rosflight_msgs::DisturbInput::Request  &req,
                                  rosflight_msgs::DisturbInput::Response &res);
  void update_signal();
  void load_param();

  Signal signal_ [8];
  double arm_time_out_time_;
  int output_type_;
  int trigger_channel_;
};

#endif // AUTO_EXCITATION_H
