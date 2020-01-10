#include <ros/ros.h>
#include <dfti2/dfti.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "dfti");
  dfti dfti_node;
  ros::spin();
}


dfti::dfti()
{
  auto currentTime = std::chrono::system_clock::now();
  std::time_t currentTime2 = std::chrono::system_clock::to_time_t(currentTime);
  logName_ = format("~/DFTI_ "+std::string(std::ctime(&currentTime2))+".csv");
  // std::cout << logName_;
  // system(("echo \"" + logName_ + "\"").c_str());
  // system(("touch " + logName_).c_str());
  // system(("echo \"ID,type,time,data\" >> " + logName_).c_str());
  logFile_.open(logName_);
  logFile_ << "ID,type,time,data\n";
  sub_ = nh_.subscribe("dfti_data", 1000, &dfti::dataCallback, this);
  ROS_INFO("DFTI READY: now logging data.");

}

void dfti::dataCallback(const dfti2::dftiData::ConstPtr& msg)
{
  ros::WallTime start_, end_;

  start_ = ros::WallTime::now();

  // system(("echo \""+std::to_string(ID_)+","+msg->type+","+std::to_string((double)msg->header.stamp.sec+((double)msg->header.stamp.nsec)/1000000000.0)+","+std::to_string(msg->data)+"\" >> " + logName_).c_str());
  logFile_ << ID_ << "," << msg->type << "," << ((double)msg->header.stamp.sec+((double)msg->header.stamp.nsec)/1000000000.0) << "," << msg->data << "\n";
  ID_++;

  end_ = ros::WallTime::now();

  double execution_time = (end_ - start_).toNSec() * 1e-6;
  ROS_INFO_STREAM("Execution time (ms): " << execution_time);
  // ROS_INFO("%s",.c_str());
}

// Function to remove all spaces from a given string
std::string format(std::string str)
{
    str.erase(remove(str.begin(), str.end(), ' '), str.end());
    str.erase(remove(str.begin(), str.end(), ':'), str.end());
    str.erase(remove(str.begin(), str.end(), '\n'), str.end());
    return str;
}
