#ifndef __ACTIVE_RADAR_NODELET_H__
#define __ACTIVE_RADAR_NODELET_H__

#include <ros/package.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/String.h>

#include <mrs_lib/param_loader.h>

namespace active_radar {
class ActiveRadarNodelet : public nodelet::Nodelet {
protected:
  // Here's our node handle!
  ros::NodeHandle nh;

  // Init publisher and subscribers here
  ros::Publisher pub;
  ros::Subscriber sub;

  std::string m_node_name;

  // These will be useful in the class implementation
  std_msgs::String message;
  unsigned int count = 0;

  // UWB related settings
  int uwb_mac_addr;
  int uwb_pan_id;

public:
  virtual void onInit();
  void msgCB(const std_msgs::StringConstPtr &msg);
};
} // namespace active_radar

#endif  // __ACTIVE_RADAR_NODELET_H__
