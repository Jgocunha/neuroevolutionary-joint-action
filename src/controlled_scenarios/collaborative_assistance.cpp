// src/collaborative_assistance_scenario.cpp
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class CollaborativeAssistanceScenario : public rclcpp::Node
{
public:
  CollaborativeAssistanceScenario()
  : rclcpp::Node("collaborative_assistance_scenario")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    pub_obj1_small_ = create_publisher<std_msgs::msg::Bool>("object1_small_presence", qos);
    pub_obj3_small_ = create_publisher<std_msgs::msg::Bool>("object3_small_presence", qos);
    pub_obj2_large_ = create_publisher<std_msgs::msg::Bool>("object2_large_presence", qos);
    pub_hand_pos_   = create_publisher<std_msgs::msg::Float64>("hand_position", qos);

    // Publish initial state immediately: all objects true, hand_position invalid (101)
    publish_bool(pub_obj1_small_, true, "object1_small_presence");
    publish_bool(pub_obj3_small_, true, "object3_small_presence");
    publish_bool(pub_obj2_large_, true, "object2_large_presence");
    publish_hand(101.0);

    // Run the scripted sequence in a dedicated thread
    worker_ = std::thread([this]{ this->run_script(); });
  }

  ~CollaborativeAssistanceScenario() override
  {
    running_ = false;
    if (worker_.joinable()) worker_.join();
  }

private:
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
    const bool valid = (value >= 1.0) && (value <= 100.0);
    RCLCPP_INFO(get_logger(), "hand_position -> %.3f (%s)", value, valid ? "valid" : "invalid");
  }

  void sleep_or_stop(std::chrono::milliseconds d)
  {
    // Sleep in small chunks so we can exit quickly if the node is shutting down
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
    // 1) wait 10 s, object3_small -> false
    sleep_or_stop(10s);
    if (!running_) return;
    publish_bool(pub_obj3_small_, false, "object3_small_presence");
    
    // 2) wait 10 s, object1_small -> false
    sleep_or_stop(20s);
    if (!running_) return;
    publish_bool(pub_obj1_small_, false, "object1_small_presence");

    // 3) wait 20 s, hand -> 50 (valid)
    sleep_or_stop(15s);
    if (!running_) return;
    publish_hand(50.0);

    //    wait 2 s, hand -> 101 (invalid)
    sleep_or_stop(1500ms);
    if (!running_) return;
    publish_hand(101.0);

    // 4) after 5 s, hand -> 50 (valid)
    sleep_or_stop(5s);
    if (!running_) return;
    publish_hand(50.0);

    //    wait 2 s, hand -> 101 (invalid)
    sleep_or_stop(5s);
    if (!running_) return;
    publish_hand(101.0);

    // 5) wait 5 s, object2_large -> false
    sleep_or_stop(5s);
    if (!running_) return;
    publish_bool(pub_obj2_large_, false, "object2_large_presence");

    RCLCPP_INFO(get_logger(), "Scenario complete. Node will keep running until shutdown.");
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_obj1_small_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_obj3_small_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_obj2_large_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_hand_pos_;

  std::thread worker_;
  std::atomic<bool> running_{true};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CollaborativeAssistanceScenario>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
