// src/collaborative_assistance_scenario_v2.cpp
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

// NEW: scene_objects messages
#include "kuka_lbr_iiwa14_marlab/msg/scene_object.hpp"
#include "kuka_lbr_iiwa14_marlab/msg/scene_objects.hpp"

using namespace std::chrono_literals;

class CollaborativeAssistanceScenarioV2 : public rclcpp::Node
{
public:
  CollaborativeAssistanceScenarioV2()
  : rclcpp::Node("anticipatory_action")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    pub_scene_    = create_publisher<kuka_lbr_iiwa14_marlab::msg::SceneObjects>("scene_objects", qos);
    pub_hand_pos_ = create_publisher<std_msgs::msg::Float64>("hand_position", qos);

    // (1) Start: no objects present, hand invalid (101)
    obj_small_1_  = false;  // 's' @ 10
    obj_small_2_  = false;  // 's' @ 50
    obj_large_30_ = false;  // 'l' @ 30
    publish_scene();
    publish_hand(101.0);

    // Script runs on a worker thread
    worker_ = std::thread([this]{ this->run_script(); });
  }

  ~CollaborativeAssistanceScenarioV2() override
  {
    running_ = false;
    if (worker_.joinable()) worker_.join();
  }

private:
  // ---- publishers ----
  rclcpp::Publisher<kuka_lbr_iiwa14_marlab::msg::SceneObjects>::SharedPtr pub_scene_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr                     pub_hand_pos_;

  // ---- presence flags ----
  bool obj_small_1_{false};   // 's' at 10
  bool obj_small_2_{false};   // 's' at 50
  bool obj_large_30_{false};  // 'l' at 30

  std::thread       worker_;
  std::atomic<bool> running_{true};

  // Build & publish the current scene
  void publish_scene()
  {
    using kuka_lbr_iiwa14_marlab::msg::SceneObject;
    using kuka_lbr_iiwa14_marlab::msg::SceneObjects;

    SceneObjects msg;
    msg.objects.reserve(3);

    auto add = [&](char t, double pos) {
      SceneObject o;
      o.type = std::string(1, t);
      o.position = pos;
      msg.objects.push_back(std::move(o));
    };

    if (obj_small_1_)  add('s', 10.0);
    if (obj_small_2_)  add('s', 50.0);
    if (obj_large_30_) add('l', 30.0);

    pub_scene_->publish(msg);
    RCLCPP_INFO(get_logger(), "scene_objects -> [%s%s%s]",
      obj_small_1_  ? "s@10 " : "",
      obj_small_2_  ? "s@50 " : "",
      obj_large_30_ ? "l@30"  : "");
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
    const int total_steps = std::max(1, static_cast<int>(std::round((duration.count()/1000.0) * rate_hz)));
    const double delta = to - from;

    auto start = std::chrono::steady_clock::now();
    for (int i = 0; running_ && i <= total_steps; ++i) {
      double alpha = static_cast<double>(i) / static_cast<double>(total_steps);
      double val = from + alpha * delta;
      publish_hand(val);

      auto next_tick = start + std::chrono::milliseconds(static_cast<int>(std::round((i+1) * 1000.0 / rate_hz)));
      while (running_ && std::chrono::steady_clock::now() < next_tick) {
        std::this_thread::sleep_for(2ms);
      }
    }
  }

  // ===== scenario script =====
  void run_script()
  {
    // Tunable durations for the sweeps
    const auto  sweep_duration = 3s;    // time for each 50->10 or 10->50 sweep
    const double start_pos = 50.0;
    const double end_pos   = 10.0;

    // (2) wait 5 s → all objects present, then hand at 50
    sleep_or_stop(5s);
    if (!running_) return;
    obj_small_1_ = true;  // s@10
    obj_small_2_ = true;  // s@50
    obj_large_30_= true;  // l@30
    publish_scene();
    sleep_or_stop(1s);
    publish_hand(start_pos);
    sleep_or_stop(2s);

    // (3) 50 -> 10 smoothly
    move_hand_linear(start_pos, end_pos, sweep_duration);
    if (!running_) return;
    sleep_or_stop(2s);

    // (4) 10 -> 50 smoothly
    move_hand_linear(end_pos, start_pos, sweep_duration);
    if (!running_) return;
    sleep_or_stop(1s);

    // (5) repeat two more cycles
    for (int cycle = 0; running_ && cycle < 2; ++cycle) {
      move_hand_linear(start_pos, end_pos, sweep_duration);
      if (!running_) return;
      sleep_or_stop(500ms);

      move_hand_linear(end_pos, start_pos, sweep_duration);
      if (!running_) return;
      sleep_or_stop(500ms);
    }

    // (6) stabilize over s@50 for ~3s
    publish_hand(start_pos);
    sleep_or_stop(3s);
    if (!running_) return;

    // (7) remove small object @50
    obj_small_2_ = false;
    publish_scene();

    // (8) after 1s set hand invalid
    sleep_or_stop(1s);
    if (!running_) return;
    publish_hand(101.0); // invalid

    // (9) after 11s remove small object @10
    sleep_or_stop(11s);
    if (!running_) return;
    obj_small_1_ = false;
    publish_scene();

    // (10) after 35s put hand over position 30
    sleep_or_stop(35s);
    publish_hand(30.0);

    // (11) after 15s set hand invalid
    sleep_or_stop(15s);
    if (!running_) return;
    publish_hand(101.0); // invalid

    // (12) finally remove the large object @30
    obj_large_30_ = false;
    publish_scene();

    RCLCPP_INFO(get_logger(), "Scenario v2 complete. Node will keep running until shutdown.");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CollaborativeAssistanceScenarioV2>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
