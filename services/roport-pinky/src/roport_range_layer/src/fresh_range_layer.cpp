// Reuse the installed Nav2 Jazzy sensor model; restore current only on valid data.
#include <cmath>
#include <deque>
#include <memory>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav2_costmap_2d/range_sensor_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace roport_range_layer
{
class FreshRangeLayer : public nav2_costmap_2d::RangeSensorLayer
{
public:
  void clearArea(int start_x, int start_y, int end_x, int end_y, bool invert) override
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getMutex());
    RangeSensorLayer::clearArea(start_x, start_y, end_x, end_y, invert);
    // Costmap clearing uses 255; the range model interprets that as probability 1.
    // Restore its neutral prior, so subsequent free observations can clear echoes.
    for (unsigned int y = 0; y < getSizeInCellsY(); ++y) {
      for (unsigned int x = 0; x < getSizeInCellsX(); ++x) {
        const bool inside = static_cast<int>(x) > start_x &&
          static_cast<int>(x) < end_x && static_cast<int>(y) > start_y &&
          static_cast<int>(y) < end_y;
        if (inside != invert) {
          setCost(x, y, default_value_);
        }
      }
    }
    std::lock_guard<std::mutex> buffer_lock(range_message_mutex_);
    range_msgs_buffer_.clear();
    buffered_readings_ = 0;
  }

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
    declareParameter("observation_keep_time", rclcpp::ParameterValue(0.0));
    node->get_parameter(name_ + ".observation_keep_time", observation_keep_time_);
    if (!std::isfinite(observation_keep_time_) || observation_keep_time_ < 0.0) {
      throw std::invalid_argument("observation_keep_time must be finite and nonnegative");
    }
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
          std::lock_guard<std::mutex> lock(pending_mutex_);
          if (pending_.size() >= 64) {pending_.pop_front();}
          pending_.push_back(*msg);
        }));
    }
  }

  void updateBounds(
    double x, double y, double yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override
  {
    std::vector<sensor_msgs::msg::Range> observations;
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      for (const auto & msg : pending_) {
        const double age = (clock_->now() -
          rclcpp::Time(msg.header.stamp, clock_->get_clock_type())).seconds();
        if (age >= -0.1 && age <= 0.3) {observations.push_back(msg);}
      }
      pending_.clear();
    }
    // The base model and the support timestamps must consume the same batch.
    // Callbacks write only pending_, including when they arrive during this update.
    for (const auto & msg : observations) {
      bufferIncomingRangeMsg(std::make_shared<sensor_msgs::msg::Range>(msg));
    }
    RangeSensorLayer::updateBounds(x, y, yaw, min_x, min_y, max_x, max_y);
    if (observation_keep_time_ > 0.0) {
      for (const auto & msg : observations) {
        geometry_msgs::msg::PointStamped origin, endpoint, world_origin, world_endpoint;
        origin.header = endpoint.header = msg.header;
        endpoint.point.x = msg.range;
        if (!tf_->canTransform(global_frame_, msg.header.frame_id,
            tf2_ros::fromMsg(msg.header.stamp), tf2::durationFromSec(0.0)))
        {
          continue;
        }
        try {
          tf_->transform(origin, world_origin, global_frame_);
          tf_->transform(endpoint, world_endpoint, global_frame_);
        } catch (const tf2::TransformException &) {
          continue;
        }
        const double theta = std::atan2(world_endpoint.point.y - world_origin.point.y,
            world_endpoint.point.x - world_origin.point.x);
        const double stamp = rclcpp::Time(msg.header.stamp, clock_->get_clock_type()).seconds();
        const double margin = std::sqrt(2.0) * resolution_ + 2.0 * resolution_ * msg.range;
        for (unsigned int iy = 0; iy < getSizeInCellsY(); ++iy) {
          for (unsigned int ix = 0; ix < getSizeInCellsX(); ++ix) {
            if (getCost(ix, iy) <= to_cost(mark_threshold_)) {continue;}
            double wx, wy;
            mapToWorld(ix, iy, wx, wy);
            const double dx = wx - world_origin.point.x, dy = wy - world_origin.point.y;
            const double distance = std::hypot(dx, dy);
            const double delta = std::atan2(std::sin(std::atan2(dy, dx) - theta),
                std::cos(std::atan2(dy, dx) - theta));
            if (std::abs(distance - msg.range) <= margin &&
              std::abs(delta) <= msg.field_of_view / 2.0 +
                std::atan2(resolution_, std::max(distance, resolution_)))
            {
              observed_at_[worldKey(wx, wy)] = stamp;
            }
          }
        }
      }
    }
    if (expireObservations(clock_->now().seconds())) {
      *min_x = std::min(*min_x, getOriginX());
      *min_y = std::min(*min_y, getOriginY());
      *max_x = std::max(*max_x, getOriginX() + getSizeInMetersX());
      *max_y = std::max(*max_y, getOriginY() + getSizeInMetersY());
    }
    // Jazzy leaves !current latched after a no-readings timeout. These fields
    // change only after the base layer has successfully transformed a reading.
    const double age = (clock_->now() - last_reading_time_).seconds();
    if (enabled_ && buffered_readings_ > 0 && age >= 0.0 &&
      (no_readings_timeout_ == 0.0 || age <= no_readings_timeout_))
    {
      current_ = true;
    }
  }

protected:
  double observation_keep_time_ = 0.0;
  std::map<std::pair<int, int>, double> observed_at_;
  std::mutex pending_mutex_;
  std::deque<sensor_msgs::msg::Range> pending_;

  std::pair<int, int> worldKey(double x, double y) const
  {
    return {static_cast<int>(std::floor(x / resolution_)),
      static_cast<int>(std::floor(y / resolution_))};
  }

  bool expireObservations(double now)
  {
    if (observation_keep_time_ == 0.0) {return false;}
    bool expired = false;
    for (unsigned int y = 0; y < getSizeInCellsY(); ++y) {
      for (unsigned int x = 0; x < getSizeInCellsX(); ++x) {
        if (getCost(x, y) <= to_cost(mark_threshold_)) {continue;}
        double wx, wy;
        mapToWorld(x, y, wx, wy);
        auto it = observed_at_.find(worldKey(wx, wy));
        if (it == observed_at_.end() || now - it->second > observation_keep_time_) {
          setCost(x, y, default_value_);
          expired = true;
        }
      }
    }
    for (auto it = observed_at_.begin(); it != observed_at_.end();) {
      if (now - it->second > observation_keep_time_) {it = observed_at_.erase(it);}
      else {++it;}
    }
    return expired;
  }
};
}  // namespace roport_range_layer
PLUGINLIB_EXPORT_CLASS(roport_range_layer::FreshRangeLayer, nav2_costmap_2d::Layer)
