#ifndef __ACTIVE_RADAR_NODELET_H__
#define __ACTIVE_RADAR_NODELET_H__

#include <cstdint>
#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <thread>
#include <unordered_map>
#include <mrs_lib/param_loader.h>
#include <boost/container/flat_map.hpp>

#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>

#include "ranging_client.h"

namespace active_radar {

// Structure representing a scheduled task.
struct ScheduledMsg {
  std::chrono::steady_clock::time_point time;
  uint16_t target_id;

  std::pair<std::vector<uint8_t>, uint64_t> tx_data;

  // Comparator: tasks with earlier time should be processed first.
  bool operator>(const ScheduledMsg &other) const {
      return time > other.time;
  }
};

class ActiveRadarNodelet : public nodelet::Nodelet {
protected:
  bool running;

  // Here's our node handle!
  ros::NodeHandle nh;

  // Init publisher and subscribers here
  ros::Publisher range_pub;

  std::string m_node_name;

  uint64_t last_broadcast_ts;

  // UWB related settings
  int uwb_mac_addr;
  int uwb_pan_id;
  bool requests;

  double correction = 0;

  int uwb_fd;

  std::priority_queue<ScheduledMsg, std::vector<ScheduledMsg>, std::greater<>> tasks_;
  std::mutex queue_mutex;
  std::condition_variable cond_var;

  std::thread recv_thread;

  std::thread send_thread;

  boost::container::flat_map<uint16_t, class RangingClient> clients;

public:
  // Constructor and destructor
  ActiveRadarNodelet();
  ~ActiveRadarNodelet();

  // Nodelet initialization called by the Nodelet manager
  virtual void onInit();
  
  // Set up the UWB socket
  bool initUWB();
  
  // Set time-sensitive networking settings for the UWB
  bool setTSN();
  
  void enqueueMsg(uint16_t target_id, std::pair<std::vector<uint8_t>, uint64_t> tx_data, int delay_ms = 0);
  
  // Threads
  void sendThread();

  void recvThread();

  void rangeCB(uint16_t id, double range, double std_dev);

  void msgCB(const std_msgs::StringConstPtr &msg);
};
} // namespace active_radar

#endif // __ACTIVE_RADAR_NODELET_H__
