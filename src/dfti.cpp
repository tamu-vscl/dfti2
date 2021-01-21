#include <ros/ros.h>
#include <dfti2/dfti.h>
#include <string>
#include <vector>
#include <filesystem>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "dfti");
  dfti dfti_node;
  ros::spin();
  dfti_node.logFile_.close();
}


dfti::dfti()
{
  // Where to store files
  std::string path = "../data";

  // Find old files so we don't overwrite them
  vector<int> old_files;
  for (const auto & entry : fs::directory_iterator(path)) old_files.push_back(entry.path().stem());
  int file_count = 1;
  while(std::find(old_files.begin(), old_files.end(), file_count) == old_files.end()) file_count++;
  logName_ = format(path + "/DFTI_run_ " + std::string(file_count) + ".csv");

  // Create file
  ROS_INFO("Creating DFTI log file: %s",logName_.c_str());
  logFile_.open(logName_);
  logFile_ << std::fixed;
  logFile_ << "ID,type,time,data\n";
  sub_ = nh_.subscribe("dfti_data", 1000, &dfti::dataCallback, this);
  ROS_INFO("DFTI READY: Now logging data.");
}

void dfti::dataCallback(const dfti2::dftiData::ConstPtr& msg)
{
  logFile_ << ID_ << "," << msg->type << "," << ((double)msg->header.stamp.sec+((double)msg->header.stamp.nsec)/1000000000.0) << "," << msg->data << "\n";
  ID_++;
}
