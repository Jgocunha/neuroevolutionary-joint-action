// src/collaborative_assistance_scenario.cpp
#include <chrono>
#include <memory>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

// NEW msgs
#include "kuka_lbr_iiwa14_marlab/msg/scene_object.hpp"
#include "kuka_lbr_iiwa14_marlab/msg/scene_objects.hpp"

using namespace std::chrono_literals;

class CollaborativeAssistanceScenario : public rclcpp::Node
{
public:
  CollaborativeAssistanceScenario()
  : rclcpp::Node("collaborative_assistance_scenario")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    pub_scene_   = create_publisher<kuka_lbr_iiwa14_marlab::msg::SceneObjects>("scene_objects", qos);
    pub_hand_pos_= create_publisher<std_msgs::msg::Float64>("hand_position", qos);

    // Initial state: all objects present; hand invalid (61)
    obj_small_1_ = false;   // 's' @ 10
    obj_small_2_ = true;   // 's' @ 50   (formerly "object3_small")
    obj_large_30_= true;   // 'l' @ 30
    publish_scene();
    publish_hand(61.0);

    // Run the scripted sequence in a dedicated thread
    worker_ = std::thread([this]{ this->run_script(); });
  }

  ~CollaborativeAssistanceScenario() override
  {
    running_ = false;
    if (worker_.joinable()) worker_.join();
  }

private:
  // --- Publishers ---
  rclcpp::Publisher<kuka_lbr_iiwa14_marlab::msg::SceneObjects>::SharedPtr pub_scene_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_hand_pos_;

  // --- Current object presence flags ---
  bool obj_small_1_{false};   // 's' at 10
  bool obj_small_2_{false};   // 's' at 50
  bool obj_large_30_{false};  // 'l' at 30

  std::thread worker_;
  std::atomic<bool> running_{true};

  // Build & publish scene based on current flags
  void publish_scene()
  {
    using kuka_lbr_iiwa14_marlab::msg::SceneObject;
    using kuka_lbr_iiwa14_marlab::msg::SceneObjects;

    SceneObjects msg;
    msg.objects.reserve(3);

    auto push = [&](char type, double pos){
      SceneObject o;
      o.type = std::string(1, type);
      o.position = pos;
      msg.objects.push_back(std::move(o));
    };

    if (obj_small_1_)  push('s', 10.0);
    if (obj_small_2_)  push('s', 50.0);
    if (obj_large_30_) push('l', 30.0);

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

  void run_script()
  {
    sleep_or_stop(500ms);
    if (!running_) return;
    obj_small_1_ = true; // 's' @ 10
    publish_scene();

    // 1) wait 18 s, small object 2 (at 50) -> false
    sleep_or_stop(18s);
    if (!running_) return;
    obj_small_2_ = false;
    publish_scene();

    // 2) wait 42 s, small object 1 (at 10) -> false
    sleep_or_stop(42s);
    if (!running_) return;
    obj_small_1_ = false;
    publish_scene();

    // 3) wait 40 s, hand -> 30 (valid)
    sleep_or_stop(40s);
    if (!running_) return;
    publish_hand(30.0);

    //    wait 2 s, hand -> 61 (invalid)
    sleep_or_stop(2s);
    if (!running_) return;
    publish_hand(61.0);

    // 4) after 5 s, hand -> 30 (valid)
    sleep_or_stop(5s);
    if (!running_) return;
    publish_hand(30.0);

    //    wait 12 s, hand -> 61 (invalid)
    sleep_or_stop(12s);
    if (!running_) return;
    publish_hand(61.0);

    // 5) wait 3 s, large object (at 30) -> false
    sleep_or_stop(3s);
    if (!running_) return;
    obj_large_30_ = false;
    publish_scene();

    RCLCPP_INFO(get_logger(), "Scenario complete. Node will keep running until shutdown.");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CollaborativeAssistanceScenario>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
