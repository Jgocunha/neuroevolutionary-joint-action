// src/collaborative_assistance_scenario_v2.cpp
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class CollaborativeAssistanceScenarioV2 : public rclcpp::Node
{
public:
  CollaborativeAssistanceScenarioV2()
  : rclcpp::Node("anticipatory_action")
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

  ~CollaborativeAssistanceScenarioV2() override
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

  // // ===== scenario script =====
  // void run_script() // vel scale 1.0
  // {
  //   // Tunable durations for the sweeps (choose sensible defaults)
  //   const auto sweep_duration = 2s;   // time for each 10->50 or 50->10 sweep
  //   const double start_pos = 10.0; // 
  //   const double end_pos   = 50.0; // 

  //   // (2) wait 5 s → all objects present, hand at 10
  //   sleep_or_stop(5s);
  //   if (!running_) return;
  //   publish_bool(pub_obj1_small_, true,  "object1_small_presence");
  //   publish_bool(pub_obj3_small_, true,  "object3_small_presence");
  //   publish_bool(pub_obj2_large_, true,  "object2_large_presence");
  //   publish_hand(start_pos);
  //   sleep_or_stop(2s);

  //   // (3) 10 -> 50 smoothly
  //   move_hand_linear(start_pos, end_pos, sweep_duration);
  //   if (!running_) return;
  //   sleep_or_stop(2s);

  //   // (4) 50 -> 10 smoothly
  //   move_hand_linear(end_pos, start_pos, sweep_duration);
  //   if (!running_) return;

  //   sleep_or_stop(1s);

  //   // (5) repeat steps (3) and (4) two more times (total 3 complete cycles)
  //   for (int cycle = 0; running_ && cycle < 2; ++cycle) {
  //     move_hand_linear(start_pos, end_pos, sweep_duration);
  //     if (!running_) return;
  //     sleep_or_stop(500ms);

  //     move_hand_linear(end_pos, start_pos, sweep_duration);
  //     if (!running_) return;
  //     sleep_or_stop(500ms);
      
  //   }

  //   // (6) stabilize over object 3 (position 80) for 5 seconds
  //   publish_hand(start_pos);
  //   sleep_or_stop(2s);
  //   if (!running_) return;

  //   // (7) remove object 3 presence
  //   publish_bool(pub_obj3_small_, false, "object3_small_presence");

  //   // (8) after 2 seconds put hand position to invalid (101)
  //   sleep_or_stop(2s);
  //   if (!running_) return;
  //   publish_hand(101.0); // invalid

  //   // (9) remove object 1 presence after 3 seconds
  //   sleep_or_stop(5s);
  //   if (!running_) return;
  //   publish_bool(pub_obj1_small_, false, "object1_small_presence");

  //   sleep_or_stop(10s);
  //   // (10) put hand over position 30
  //   publish_hand(30.0);

  //   // (11) remove hand after 10 seconds (invalid)
  //   sleep_or_stop(7s);
  //   if (!running_) return;
  //   publish_hand(101.0); // invalid

  //   // // (12) object 2 false after 5 seconds
  //   // sleep_or_stop(5s);
  //   // if (!running_) return;
  //   publish_bool(pub_obj2_large_, false, "object2_large_presence");

  //   RCLCPP_INFO(get_logger(), "Scenario v2 complete. Node will keep running until shutdown.");
  // }

    // ===== scenario script =====
  void run_script() // vel scale 1.0
  {
    // Tunable durations for the sweeps (choose sensible defaults)
    const auto sweep_duration = 3s;   // time for each 10->50 or 50->10 sweep
    const double start_pos = 50.0; // 
    const double end_pos   = 10.0; // 

    // (2) wait 5 s → all objects present, hand at 10
    sleep_or_stop(5s);
    if (!running_) return;
    publish_bool(pub_obj1_small_, true,  "object1_small_presence");
    publish_bool(pub_obj3_small_, true,  "object3_small_presence");
    publish_bool(pub_obj2_large_, true,  "object2_large_presence");
    sleep_or_stop(1s);
    publish_hand(start_pos);
    sleep_or_stop(2s);

    // (3) 10 -> 50 smoothly
    move_hand_linear(start_pos, end_pos, sweep_duration);
    if (!running_) return;
    sleep_or_stop(2s);

    // (4) 50 -> 10 smoothly
    move_hand_linear(end_pos, start_pos, sweep_duration);
    if (!running_) return;

    sleep_or_stop(1s);

    // (5) repeat steps (3) and (4) two more times (total 3 complete cycles)
    for (int cycle = 0; running_ && cycle < 2; ++cycle) {
      move_hand_linear(start_pos, end_pos, sweep_duration);
      if (!running_) return;
      sleep_or_stop(500ms);

      move_hand_linear(end_pos, start_pos, sweep_duration);
      if (!running_) return;
      sleep_or_stop(500ms);
      
    }

    // (6) stabilize over object 3 (position 80) for 5 seconds
    publish_hand(start_pos);
    sleep_or_stop(3s);
    if (!running_) return;

    // (7) remove object 3 presence
    publish_bool(pub_obj3_small_, false, "object3_small_presence");

    // (8) after 2 seconds put hand position to invalid (101)
    sleep_or_stop(1s);
    if (!running_) return;
    publish_hand(101.0); // invalid

    // (9) remove object 1 presence after 3 seconds
    sleep_or_stop(11s);
    if (!running_) return;
    publish_bool(pub_obj1_small_, false, "object1_small_presence");

    sleep_or_stop(35s);
    // (10) put hand over position 30
    publish_hand(30.0);

    // (11) remove hand after 10 seconds (invalid)
    sleep_or_stop(15s);
    if (!running_) return;
    publish_hand(101.0); // invalid

    // // (12) object 2 false after 5 seconds
    // sleep_or_stop(5s);
    // if (!running_) return;
    publish_bool(pub_obj2_large_, false, "object2_large_presence");

    RCLCPP_INFO(get_logger(), "Scenario v2 complete. Node will keep running until shutdown.");
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
  auto node = std::make_shared<CollaborativeAssistanceScenarioV2>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
