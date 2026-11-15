// Reuse the installed Nav2 Jazzy sensor model; restore current only on valid data.
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include "nav2_costmap_2d/range_sensor_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace roport_range_layer
{
class FreshRangeLayer : public nav2_costmap_2d::RangeSensorLayer
{
public:
  void onInitialize() override
  {
    RangeSensorLayer::onInitialize();
    // The base callback does not reject stale or NaN VARIABLE observations.
    // Replace its subscriptions, retaining its sensor model and cost merging.
    range_subs_.clear();
    {
      std::lock_guard<std::mutex> lock(range_message_mutex_);
      range_msgs_buffer_.clear();
    }
    auto node = node_.lock();
    const auto topics = node->get_parameter(name_ + ".topics").as_string_array();
    for (const auto & topic : topics) {
      range_subs_.push_back(node->create_subscription<sensor_msgs::msg::Range>(
        topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Range::SharedPtr msg) {
          if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000u ||
            msg->header.frame_id.empty() || !std::isfinite(msg->min_range) ||
            !std::isfinite(msg->max_range) || !std::isfinite(msg->range) ||
            !std::isfinite(msg->field_of_view) || msg->min_range <= 0.0 ||
            msg->max_range <= msg->min_range || msg->range < msg->min_range ||
            msg->range > msg->max_range || msg->field_of_view <= 0.0 ||
            msg->field_of_view >= 3.141592653589793)
          {
            return;
          }
          const double age = (clock_->now() -
            rclcpp::Time(msg->header.stamp, clock_->get_clock_type())).seconds();
          if (age < -0.1 || age > 0.3) {
            return;
          }
          bufferIncomingRangeMsg(msg);
        }));
    }
  }

  void updateBounds(
    double x, double y, double yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override
  {
    RangeSensorLayer::updateBounds(x, y, yaw, min_x, min_y, max_x, max_y);
    // Jazzy leaves !current latched after a no-readings timeout. These fields
    // change only after the base layer has successfully transformed a reading.
    const double age = (clock_->now() - last_reading_time_).seconds();
    if (enabled_ && buffered_readings_ > 0 && age >= 0.0 &&
      (no_readings_timeout_ == 0.0 || age <= no_readings_timeout_))
    {
      current_ = true;
    }
  }
};
}  // namespace roport_range_layer
PLUGINLIB_EXPORT_CLASS(roport_range_layer::FreshRangeLayer, nav2_costmap_2d::Layer)
