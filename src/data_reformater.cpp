#include "ros/ros.h"
#include "dfti2/dftiData.h"
#include "nav_msgs/Odometry.h"
#include "mavros_msgs/RCOut.h"

class data_reformater_node
{
  public:
    data_reformater_node();
  private:
    ros::NodeHandle n;
    ros::Subscriber odom_sub;
    ros::Subscriber rc_sub;
    ros::Publisher data_pub;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& in_msg);
    void rcCallback(const mavros_msgs::RCOut::ConstPtr& in_msg);
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "da<build_depend>dfti2</build_depend>ta_reformater");
  data_reformater_node start;
  ros::spin();
  return 0;
}

data_reformater_node::data_reformater_node()
{
  data_pub = n.advertise<dfti2::dftiData>("dfti_data",1000);
  odom_sub = n.subscribe("/mavros/local_position/odom",1000,&data_reformater_node::odomCallback,this);
  rc_sub = n.subscribe("/mavros/rc/out",1000,&data_reformater_node::rcCallback,this);
}

void data_reformater_node::odomCallback(const nav_msgs::Odometry::ConstPtr& in_msg)
{
  dfti2::dftiData out_msg;
  out_msg.header = in_msg->header;

  out_msg.type = "x";
  out_msg.data = in_msg->pose.pose.position.y;
  data_pub.publish(out_msg);
  out_msg.type = "y";
  out_msg.data = in_msg->pose.pose.position.x;
  data_pub.publish(out_msg);
  out_msg.type = "z";
  out_msg.data = -in_msg->pose.pose.position.z;
  data_pub.publish(out_msg);

  out_msg.type = "q0";
  out_msg.data = in_msg->pose.pose.orientation.w;
  data_pub.publish(out_msg);
  out_msg.type = "q1";
  out_msg.data = in_msg->pose.pose.orientation.y;
  data_pub.publish(out_msg);
  out_msg.type = "q2";
  out_msg.data = in_msg->pose.pose.orientation.x;
  data_pub.publish(out_msg);
  out_msg.type = "q3";
  out_msg.data = -in_msg->pose.pose.orientation.z;
  data_pub.publish(out_msg);

  out_msg.type = "u";
  out_msg.data = in_msg->twist.twist.linear.x;
  data_pub.publish(out_msg);
  out_msg.type = "v";
  out_msg.data = -in_msg->twist.twist.linear.y;
  data_pub.publish(out_msg);
  out_msg.type = "w";
  out_msg.data = -in_msg->twist.twist.linear.z;
  data_pub.publish(out_msg);

  out_msg.type = "p";
  out_msg.data = in_msg->twist.twist.angular.x;
  data_pub.publish(out_msg);
  out_msg.type = "q";
  out_msg.data = -in_msg->twist.twist.angular.y;
  data_pub.publish(out_msg);
  out_msg.type = "r";
  out_msg.data = -in_msg->twist.twist.angular.z;
  data_pub.publish(out_msg);
}

void data_reformater_node::rcCallback(const mavros_msgs::RCOut::ConstPtr& in_msg)
{
  dfti2::dftiData out_msg;
  out_msg.header = in_msg->header;
  for(int i = 0; i<4; i++)
  {
    out_msg.type = "M" + std::to_string(i+1);
    out_msg.data = in_msg->channels[i];
    data_pub.publish(out_msg);
  }
}
