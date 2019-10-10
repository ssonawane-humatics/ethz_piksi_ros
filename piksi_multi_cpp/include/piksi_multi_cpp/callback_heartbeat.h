#ifndef PIKSI_MULTI_CPP_CALLBACK_HEARTBEAT_H_
#define PIKSI_MULTI_CPP_CALLBACK_HEARTBEAT_H_

#include <libsbp/sbp.h>
#include <ros/ros.h>
#include "piksi_multi_cpp/callback.h"

namespace piksi_multi_cpp {

class CallbackHeartbeat : public Callback {
 public:
  CallbackHeartbeat(const ros::NodeHandle& nh, const uint16_t sbp_msg_type,
                    const std::shared_ptr<sbp_state_t>& state);

 private:
  void callback(uint16_t sender_id, uint8_t len, uint8_t msg[]) override;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_CALLBACK_HEARTBEAT_H_
