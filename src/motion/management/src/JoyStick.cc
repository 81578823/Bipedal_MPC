#include "JoyStick.h"

namespace clear {
using std::placeholders::_1;

JoyStick::JoyStick(rclcpp::Node::SharedPtr nodeHandle)
    : nodeHandle_(nodeHandle) {

  joy_sub_ = nodeHandle_->create_subscription<sensor_msgs::msg::Joy>(
      "joy", rclcpp::SensorDataQoS(), std::bind(&JoyStick::joy_cb, this, _1));

  e_stop_.push(false);
  start_.push(false);

  RCLCPP_INFO(nodeHandle_->get_logger(), "JoyStick Initialized!");
  vel_cmd.setZero();
}

void JoyStick::joy_cb(const sensor_msgs::msg::Joy::SharedPtr joy_msg) const {
  joy_msg_.push(joy_msg);
}

vector3_t JoyStick::getLinearVelCmd() {
  auto msg = joy_msg_.get();
  if (msg == nullptr) {
      // vector3_t vel_cmd_t;
      // vel_cmd_t.z() = 0.0;
      // vel_cmd_t.x() = -0.2;
      // vel_cmd_t.y() = 0.0;
      // vel_cmd = 0.999 * vel_cmd + 0.001 * vel_cmd_t;
      // return vel_cmd;
    return vector3_t::Zero();
  } else {
    if (msg->axes.at(5) < 0.9) {
      vector3_t vel_cmd_t;
      vel_cmd_t.z() = 0.0;
      vel_cmd_t.x() = 0.4;
      vel_cmd_t.y() = 0.0;
      vel_cmd = 0.999 * vel_cmd + 0.001 * vel_cmd_t;
      return vel_cmd;
    } else if (msg->axes.at(2) < 0.9) {
      vector3_t vel_cmd_t;
      vel_cmd_t.z() = 0.0;
      vel_cmd_t.x() = -0.4;
      vel_cmd_t.y() = 0.0;
      vel_cmd = 0.999 * vel_cmd + 0.001 * vel_cmd_t;
      return vel_cmd;
    } else {
      vector3_t vel_cmd_t;
      vel_cmd_t.z() = 0.0;
      vel_cmd_t.x() = 0.3 * msg->axes.at(1);
      vel_cmd_t.y() = 0.1 * msg->axes.at(0);
      vel_cmd = 0.995 * vel_cmd + 0.005 * vel_cmd_t;
      return vel_cmd;
    }
  }
}

scalar_t JoyStick::getYawVelCmd() {
  const scalar_t angular_velocity_factor = 0.3;  // 降低到0.05，匹配ConvexMPC的限制
  auto msg = joy_msg_.get();
  if (msg == nullptr) {
    return 0.0;
  } else {
    scalar_t angular_velocity_t;
    angular_velocity_t = angular_velocity_factor * msg->axes.at(3);
    angular_velocity = 0.995 * angular_velocity + 0.005 * angular_velocity_t;
    return angular_velocity;
  }
}

bool JoyStick::eStop() {
  auto msg = joy_msg_.get();
  if (msg == nullptr) {
    return false;
  } else {
    return (msg->buttons.at(7) > 0);
  }
}

bool JoyStick::isStart() {
  auto msg = joy_msg_.get();  
  if (msg != nullptr) {
    if (!start_.get() && msg->buttons.at(5) > 0) {
      start_.push(true);
    }
  }
  return start_.get();
}

} // namespace clear
