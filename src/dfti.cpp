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
  system(("echo \"" + logName_ + "\"").c_str());
  system(("touch " + logName_).c_str());
  system(("echo \"ID,type,time,data\" >> " + logName_).c_str());

  sub_ = nh_.subscribe("dfti_data", 1000, &dfti::dataCallback, this);

}

void dfti::dataCallback(const dfti2::dftiData::ConstPtr& msg)
{
  system(("echo \""+std::to_string(ID_)+","+msg->type+","+std::to_string(msg->header.stamp.toSec())+","+std::to_string(msg->data)+"\" >> " + logName_).c_str());
  ID_++;
}

// Function to remove all spaces from a given string
std::string format(std::string str)
{
    str.erase(remove(str.begin(), str.end(), ' '), str.end());
    str.erase(remove(str.begin(), str.end(), ':'), str.end());
    str.erase(remove(str.begin(), str.end(), '\n'), str.end());
    return str;
}
