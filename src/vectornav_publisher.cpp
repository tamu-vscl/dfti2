#include "ros/ros.h"
#include "dfti2/dftiData.h"
#include "nav_msgs/Odometry.h"
#include <vectornav/Ins.h>
#include <vector>

class vectornav_publisher_node
{
  public:
    vectornav_publisher_node();
  private:
    ros::NodeHandle n;
    ros::Subscriber odom_sub;
    ros::Publisher data_pub;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vectornav_publisher");
  vectornav_publisher_node start;
  ros::spin();
  return 0;
}

vectornav_publisher_node::vectornav_publisher_node()
{
  data_pub = n.advertise<dfti2::dftiData>("dfti_data",1000);
  odom_sub = n.subscribe("/vectornav/Odom",1000,&vectornav_publisher_node::odomCallback,this);
}

void vectornav_publisher_node::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  dfti2::dftiData out_msg;
  out_msg.header = odom_msg->header;

  // Publish Quaterions 
  out_msg.type = "vn_qi";
  out_msg.data = odom_msg->pose.pose.orientation.x;
  data_pub.publish(out_msg);

  out_msg.type = "vn_qj";
  out_msg.data = odom_msg->pose.pose.orientation.y;
  data_pub.publish(out_msg);

  out_msg.type = "vn_qk";
  out_msg.data = odom_msg->pose.pose.orientation.z;
  data_pub.publish(out_msg);

  out_msg.type = "vn_qw";
  out_msg.data = odom_msg->pose.pose.orientation.w;
  data_pub.publish(out_msg);


  // Publish Position
  out_msg.type = "vn_x";
  out_msg.data = odom_msg->pose.pose.position.x;
  data_pub.publish(out_msg);
  
  out_msg.type = "vn_y";
  out_msg.data = odom_msg->pose.pose.position.y;
  data_pub.publish(out_msg);
  
  out_msg.type = "vn_z";
  out_msg.data = odom_msg->pose.pose.position.z;
  data_pub.publish(out_msg);


  // Publish Velocities (m/s in body frame)
  out_msg.type = "vn_u";
  out_msg.data = odom_msg->twist.twist.linear.x;
  data_pub.publish(out_msg);
  
  out_msg.type = "vn_v";
  out_msg.data = odom_msg->twist.twist.linear.y;
  data_pub.publish(out_msg);
  
  out_msg.type = "vn_w";
  out_msg.data = odom_msg->twist.twist.linear.z;
  data_pub.publish(out_msg);


  // Publish Angular Rates (deg/s)
  out_msg.type = "vn_p";
  out_msg.data = odom_msg->twist.twist.angular.x;
  data_pub.publish(out_msg);
  
  out_msg.type = "vn_q";
  out_msg.data = odom_msg->twist.twist.angular.y;
  data_pub.publish(out_msg);
  
  out_msg.type = "vn_r";
  out_msg.data = odom_msg->twist.twist.angular.z;
  data_pub.publish(out_msg);

}
