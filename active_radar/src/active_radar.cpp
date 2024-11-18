#include <socket_utils.h>
#include <mrs_lib/scope_timer.h>

#include "active_radar.h"

ros::Rate r = ros::Rate(25);

namespace active_radar {

void ActiveRadarNodelet::onInit() {
  // Init nodehandle
  this->nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  this->m_node_name = "ActiveRadar";

  // Bind subscription topic
  sub = nh.subscribe("chatter", 1, &ActiveRadarNodelet::msgCB, this);
  pub = nh.advertise<std_msgs::String>("chatter", 1);

  NODELET_INFO("NODELET READY!");

  mrs_lib::ParamLoader pl(this->nh, this->m_node_name);

  // Load parameters
  pl.loadParam("uwb_mac_addr", this->uwb_mac_addr);
  pl.loadParam("uwb_pan_id", this->uwb_pan_id);

  // print loaded parameters
  NODELET_INFO_STREAM("uwb_mac_addr: " << this->uwb_mac_addr);
    NODELET_INFO_STREAM("uwb_pan_id: " << this->uwb_pan_id);

  while (ros::ok()) {

    ros::spinOnce();
    r.sleep();

    count++;
  }
}

void ActiveRadarNodelet::msgCB(const std_msgs::String::ConstPtr &msg) {
  NODELET_INFO_STREAM(msg->data);
}
} // namespace active_radar


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(active_radar::ActiveRadarNodelet, nodelet::Nodelet)
