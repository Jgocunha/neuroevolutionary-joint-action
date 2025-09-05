// src/collaborative_assistance_scenario_v2.cpp
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class CollaborativePreferenceAction : public rclcpp::Node
{
public:
  CollaborativePreferenceAction()
  : rclcpp::Node("collaborative_preference_action")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    pub_obj1_small_ = create_publisher<std_msgs::msg::Bool>("object1_small_presence", qos);
    pub_obj3_small_ = create_publisher<std_msgs::msg::Bool>("object3_small_presence", qos);
    pub_obj2_large_ = create_publisher<std_msgs::msg::Bool>("object2_large_presence", qos);
    pub_hand_pos_   = create_publisher<std_msgs::msg::Float64>("hand_position", qos);

    // (1) Start: all objects NOT present, hand invalid (101)
    publish_bool(pub_obj1_small_, false, "object1_small_presence");
    publish_bool(pub_obj3_small_, false, "object3_small_presence");
    publish_bool(pub_obj2_large_, false, "object2_large_presence");
    publish_hand(101.0);

    // Script runs on a worker thread
    worker_ = std::thread([this]{ this->run_script(); });
  }

  ~CollaborativePreferenceAction() override
  {
    running_ = false;
    if (worker_.joinable()) worker_.join();
  }

private:
  // ===== helpers =====
  void publish_bool(const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr& pub,
                    bool value, const char* name)
  {
    std_msgs::msg::Bool msg; msg.data = value;
    pub->publish(msg);
    RCLCPP_INFO(get_logger(), "%s -> %s", name, value ? "true" : "false");
  }

  void publish_hand(double value)
  {
    std_msgs::msg::Float64 msg; msg.data = value;
    pub_hand_pos_->publish(msg);
    const bool valid = (value >= 1.0) && (value <= 60.0);
    RCLCPP_INFO(get_logger(), "hand_position -> %.3f (%s)", value, valid ? "valid" : "invalid");
  }

  void sleep_or_stop(std::chrono::milliseconds d)
  {
    auto remaining = d;
    const auto step = 50ms;
    while (running_ && remaining.count() > 0) {
      auto chunk = (remaining > step) ? step : remaining;
      std::this_thread::sleep_for(chunk);
      remaining -= chunk;
    }
  }

  // Smooth linear motion from 'from' to 'to' over 'duration', publishing at 'rate_hz'
  void move_hand_linear(double from, double to, std::chrono::milliseconds duration,
                        double rate_hz = 10.0)
  {
    if (!running_) return;
    //const double dt = 1.0 / rate_hz;
    const int total_steps = std::max(1, static_cast<int>(std::round((duration.count()/1000.0) * rate_hz)));
    const double delta = to - from;

    auto start = std::chrono::steady_clock::now();
    for (int i = 0; running_ && i <= total_steps; ++i) {
      double alpha = static_cast<double>(i) / static_cast<double>(total_steps);
      double val = from + alpha * delta;
      publish_hand(val);

      // Wait until the next tick based on wall time, to keep timing stable
      auto next_tick = start + std::chrono::milliseconds(static_cast<int>(std::round((i+1) * 1000.0 / rate_hz)));
      while (running_ && std::chrono::steady_clock::now() < next_tick) {
        std::this_thread::sleep_for(2ms);
      }
    }
  }
  
  // ===== scenario script =====
  void run_script() // vel scale 1.0
  {
    sleep_or_stop(5s);
    if (!running_) return;
    publish_bool(pub_obj1_small_, true,  "object1_small_presence");
    publish_bool(pub_obj3_small_, true,  "object3_small_presence");
    publish_bool(pub_obj2_large_, true,  "object2_large_presence");
    //sleep_or_stop(1s);
    publish_hand(30.0);
    sleep_or_stop(2s);
    if (!running_) return;

    move_hand_linear(30.0, 10.0, 2s);
    if (!running_) return;
    sleep_or_stop(1s);

    move_hand_linear(10.0, 30.0, 2s);
    if (!running_) return;
    sleep_or_stop(3s);

    move_hand_linear(30.0, 10.0, 2s);
    if (!running_) return;
    sleep_or_stop(200ms);

    move_hand_linear(10.0, 30.0, 2s);
    if (!running_) return;
    sleep_or_stop(20s);
    if (!running_) return;
    publish_bool(pub_obj2_large_, false, "object2_large_presence");
    publish_hand(101.0); // invalid

    sleep_or_stop(1s);
    if (!running_) return;
    publish_bool(pub_obj1_small_, true,  "object1_small_presence");
    publish_bool(pub_obj3_small_, true,  "object3_small_presence");

    RCLCPP_INFO(get_logger(), "collaborative_preference_action complete. Node will keep running until shutdown.");
  }

  // pubs
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    pub_obj1_small_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    pub_obj3_small_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    pub_obj2_large_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_hand_pos_;

  std::thread worker_;
  std::atomic<bool> running_{true};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CollaborativePreferenceAction>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
