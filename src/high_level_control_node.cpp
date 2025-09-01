#include <memory>
#include <thread>
#include <atomic>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"

#include "dnf_composer/application/application.h"
#include "dnf_composer/simulation/simulation_file_manager.h"
#include "dnf_composer/user_interface/main_window.h"
#include "dnf_composer/user_interface/field_metrics_window.h"
#include "dnf_composer/user_interface/element_window.h"
#include "dnf_composer/user_interface/plot_control_window.h"
#include "dnf_composer/user_interface/simulation_window.h"
#include "dnf_composer/user_interface/node_graph_window.h"
#include "dnf_composer/user_interface/plots_window.h"
#include "imgui-platform-kit/log_window.h"

class DnfNode : public rclcpp::Node 
{
public:
  DnfNode() 
    : Node("dnf_control_architecture") 
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    sub_object1_small_ = create_subscription<std_msgs::msg::Bool>(
      "object1_small_presence", qos,
      [this](std_msgs::msg::Bool::SharedPtr msg){
        object1_small_.store(msg->data, std::memory_order_relaxed);
    });

    sub_object3_small_ = create_subscription<std_msgs::msg::Bool>(
      "object3_small_presence", qos,
      [this](std_msgs::msg::Bool::SharedPtr msg){
        object3_small_.store(msg->data, std::memory_order_relaxed);
    });

    sub_object2_large_ = create_subscription<std_msgs::msg::Bool>(
      "object2_large_presence", qos,
      [this](std_msgs::msg::Bool::SharedPtr msg){
        object2_large_.store(msg->data, std::memory_order_relaxed);
    });

    sub_hand_position_ = create_subscription<std_msgs::msg::Float64>(
      "hand_position", qos,
      [this](std_msgs::msg::Float64::SharedPtr msg){
        const double v = msg->data;
        const bool valid = (v >= 1.0) && (v <= 100.0);
        hand_valid_.store(valid, std::memory_order_relaxed);
        hand_position_.store(v, std::memory_order_relaxed);
    });

    pub_target_object_ = create_publisher<std_msgs::msg::Int32>("target_object", 2);

    double hz = declare_parameter<double>("target_object_rate_hz", 2.0);
    //if (hz <= 0.0) hz = 10.0;
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(1.0 / hz));

    timer_ = create_wall_timer(period, [this]{
      std_msgs::msg::Int32 msg;
      msg.data = target_object_.load(std::memory_order_relaxed);
      pub_target_object_->publish(msg);
    });
  }

  bool object1_small() const { return object1_small_.load(std::memory_order_relaxed); }
  bool object3_small() const { return object3_small_.load(std::memory_order_relaxed); }
  bool object2_large() const { return object2_large_.load(std::memory_order_relaxed); }
  bool hand_valid() const { return hand_valid_.load(std::memory_order_relaxed); }
  double hand_position() const { return hand_position_.load(std::memory_order_relaxed); }
 
  void set_target_object(int v) { target_object_.store(v, std::memory_order_relaxed); }

private:
  std::atomic<bool> object1_small_{false}, object3_small_{false}, object2_large_{false};
  std::atomic<bool> hand_valid_{false};
  std::atomic<double> hand_position_{0.0};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      sub_object1_small_, sub_object3_small_, sub_object2_large_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_hand_position_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_target_object_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<int> target_object_{0};
};

int main(int argc, char **argv) 
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DnfNode>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  std::atomic<bool> ros_running{true};
  std::thread ros_thread([&]()
  {
    exec.spin();
    ros_running = false;
  });

  try 
  {
    using namespace dnf_composer;

    const std::shared_ptr<dnf_composer::Simulation> previous_solution = 
        std::make_shared<dnf_composer::Simulation>("packaging task control architecture");
    const dnf_composer::SimulationFileManager sfm(previous_solution, 
        std::string(OUTPUT_DIRECTORY) + "/solution 99514 generation 200 species 770 fitness 0.896474.json");
    sfm.loadElementsFromJson();
    auto visualization = std::make_shared<Visualization>(previous_solution);
    const Application app{ previous_solution, visualization };
    
    visualization->plot(
        PlotCommonParameters{
        PlotType::LINE_PLOT,
        PlotDimensions{ 0.0, 100, -20.0, 20, 1.0, 1.0},
        PlotAnnotations{ "small object location input field", "Spatial location", "Amplitude" } },
        LinePlotParameters{},
        { 
            { "nf 1", "activation" }, 
            { "gs nf 1 20.000000", "output" },
            { "gs nf 1 80.000000", "output" },
        }
    );

	visualization->plot(
        PlotCommonParameters{
        PlotType::LINE_PLOT,
        PlotDimensions{ 0.0, 100, -20.0, 20, 1.0, 1.0},
        PlotAnnotations{ "large object location input field", "Spatial location", "Amplitude" } },
        LinePlotParameters{},
        { 
            { "nf 2", "activation" }, 
            { "gs nf 2 50.000000", "output" },
        }
    );

    visualization->plot(
        PlotCommonParameters{
        PlotType::LINE_PLOT,
        PlotDimensions{ 0.0, 100, -20.0, 20, 1.0, 1.0},
        PlotAnnotations{ "hand position input field", "Spatial location", "Amplitude" } },
        LinePlotParameters{},
        { 
            { "nf 3", "activation" }, 
            { "gs nf 3 50.000000", "output" },
    });

    visualization->plot(
        PlotCommonParameters{
        PlotType::LINE_PLOT,
        PlotDimensions{ 0.0, 100, -20.0, 20, 1.0, 1.0},
        PlotAnnotations{ "hidden field", "Spatial location", "Amplitude" } },
        LinePlotParameters{},
        { 
            { "nf 5", "activation" }, 
    });

    visualization->plot(
        PlotCommonParameters{
        PlotType::LINE_PLOT,
        PlotDimensions{ 0.0, 100, -20.0, 20, 1.0, 1.0},
        PlotAnnotations{ "target robot action field", "Spatial location", "Amplitude" } },
        LinePlotParameters{},
        { 
            { "nf 4", "activation" }, 
    });

    app.addWindow<user_interface::MainWindow>();
    //app.addWindow<imgui_kit::LogWindow>();
    //app.addWindow<user_interface::FieldMetricsWindow>();
    //app.addWindow<user_interface::ElementWindow>();
    //app.addWindow<user_interface::SimulationWindow>();
    //app.addWindow<user_interface::PlotControlWindow>();
    app.addWindow<user_interface::PlotsWindow>();
    //app.addWindow<user_interface::NodeGraphWindow>();

    // Helper: apply amplitude to a GaussStimulus by element name (only in GUI thread)
    auto apply_presence = [&](const std::string& element_name, bool present) 
    {
        auto base_elem = previous_solution->getElement(element_name);
        auto gs = std::dynamic_pointer_cast<dnf_composer::element::GaussStimulus>(base_elem);
        if (!gs) {
            log(dnf_composer::tools::logger::LogLevel::ERROR,
                "Element '" + element_name + "' is not a GaussStimulus.",
                dnf_composer::tools::logger::LogOutputMode::CONSOLE);
            return;
        }
        const auto p = gs->getParameters();
        double amplitude = present ? 20.0 : 0.0;
        if (element_name == "gs nf 1 20.000000" && amplitude == 20.0)
          amplitude = amplitude + 3.0;
        if (element_name == "gs nf 1 80.000000" && amplitude == 20.0)
          amplitude = amplitude + 3.0;
        gs->setParameters({ p.width, amplitude, p.position });
    };

   // Helper: set hand stimulus parameters (only in GUI thread)
   auto apply_hand_stimulus = [&](double hand_pos, bool valid)
    {
        auto base_elem = previous_solution->getElement("gs nf 3 50.000000");
        auto gs = std::dynamic_pointer_cast<dnf_composer::element::GaussStimulus>(base_elem);
        if (!gs) {
            log(dnf_composer::tools::logger::LogLevel::ERROR,
                "Element 'gs nf 3 50.000000' is not a GaussStimulus.",
                dnf_composer::tools::logger::LogOutputMode::CONSOLE);
            return;
        }
        // When valid: width=5, amplitude=20, position=hand_pos; else amplitude=0
        if (valid) {
            gs->setParameters({ 5.0, 20.0, hand_pos });
        } else {
            const auto p = gs->getParameters();
            gs->setParameters({ p.width, 0.0, p.position });
        }
    };

   bool last_object1_small = false, last_object2_large = false, last_object3_small = false;
   bool last_hand_valid = false;
   double last_hand_pos = 0.0;
   
   auto classify_from_centroid = [](double c)->int 
    {
        auto in = [&](double center){ return (c >= center - 2.0) && (c <= center + 2.0); };
        if (in(20.0)) return 1;
        if (in(50.0)) return 2;
        if (in(80.0)) return 3;
        return 0;
    };

    // returns current classification (0..3) or 0 if invalid
    auto compute_target_object = [&]() -> int 
    {
        auto base = previous_solution->getElement("nf 4");
        auto nf4  = std::dynamic_pointer_cast<dnf_composer::element::NeuralField>(base);
        if (!nf4) {
            log(dnf_composer::tools::logger::LogLevel::ERROR,
                "Element 'nf 4' is not a NeuralField.",
                dnf_composer::tools::logger::LogOutputMode::CONSOLE);
            return 0;
        }

        const auto& bumps = nf4->getBumps();
        if (bumps.size() != 1) 
            return 0;

        const double centroid = bumps[0].centroid;
        return classify_from_centroid(centroid);
    };
    
    app.init();

    while (rclcpp::ok() && !app.hasGUIBeenClosed()) 
    {
          // read latest desired states from the node (atomics -> cheap)
          const bool cur_object1_small = node->object1_small();
          const bool cur_object2_large = node->object2_large();
          const bool cur_object3_small = node->object3_small();
          const bool cur_hand_valid = node->hand_valid();
          const double cur_hand_pos = node->hand_position();

          // Apply only when the state changed
          if (cur_object1_small != last_object1_small) {
              apply_presence("gs nf 1 20.000000", cur_object1_small);
              last_object1_small = cur_object1_small;
          }
          if (cur_object3_small != last_object3_small) {
              apply_presence("gs nf 1 80.000000", cur_object3_small);
              last_object3_small = cur_object3_small;
          }
          if (cur_object2_large != last_object2_large) {
              apply_presence("gs nf 2 50.000000", cur_object2_large);
              last_object2_large = cur_object2_large;
          }

          // Update hand stimulus when validity flips or (if valid) position changes
          if (cur_hand_valid != last_hand_valid ||
              (cur_hand_valid && cur_hand_pos != last_hand_pos)) {
              apply_hand_stimulus(cur_hand_pos, cur_hand_valid);
              last_hand_valid = cur_hand_valid;
              last_hand_pos = cur_hand_pos;
          }

          // compute classification
          int current = compute_target_object();
          // store to node; timer will publish at fixed rate
          node->set_target_object(current);

          app.step();  // render + update one frame
      }

    app.close();
  } catch (const dnf_composer::Exception& ex) {
    const std::string msg = "Exception: " + std::string(ex.what()) +
      " ErrorCode: " + std::to_string(static_cast<int>(ex.getErrorCode())) + ". ";
    log(dnf_composer::tools::logger::LogLevel::FATAL, msg,
        dnf_composer::tools::logger::LogOutputMode::CONSOLE);
  } catch (const std::exception& ex) {
    log(dnf_composer::tools::logger::LogLevel::FATAL,
        "Exception caught: " + std::string(ex.what()) + ". ",
        dnf_composer::tools::logger::LogOutputMode::CONSOLE);
  } catch (...) {
    log(dnf_composer::tools::logger::LogLevel::FATAL,
        "Unknown exception occurred. ",
        dnf_composer::tools::logger::LogOutputMode::CONSOLE);
  }

  exec.cancel();
  if (ros_thread.joinable()) ros_thread.join();
  rclcpp::shutdown();
  return 0;
}