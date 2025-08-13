#include <memory>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"

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

class DnfNode : public rclcpp::Node {
public:
  DnfNode() : Node("dnf_control_architecture") {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    sub_objs1_ = create_subscription<std_msgs::msg::Bool>(
      "objs1_presence", qos,
      [this](std_msgs::msg::Bool::SharedPtr msg){ objs1_.store(msg->data, std::memory_order_relaxed); });

    sub_objs2_ = create_subscription<std_msgs::msg::Bool>(
      "objs2_presence", qos,
      [this](std_msgs::msg::Bool::SharedPtr msg){ objs2_.store(msg->data, std::memory_order_relaxed); });

    sub_objl_ = create_subscription<std_msgs::msg::Bool>(
      "objl_presence", qos,
      [this](std_msgs::msg::Bool::SharedPtr msg){ objl_.store(msg->data, std::memory_order_relaxed); });

    pub_target_object_ = create_publisher<std_msgs::msg::Int32>("target_object", 10);
  }

  bool objs1() const { return objs1_.load(std::memory_order_relaxed); }
  bool objs2() const { return objs2_.load(std::memory_order_relaxed); }
  bool objl() const { return objl_.load(std::memory_order_relaxed); }

  void publish_target_object(int v) {
    std_msgs::msg::Int32 msg;
    msg.data = v;
    pub_target_object_->publish(msg);
  }

private:
  std::atomic<bool> objs1_{false}, objs2_{false}, objl_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_objs1_, sub_objs2_, sub_objl_;
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_target_object_;
};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // 1) Create your ROS node
  auto node = std::make_shared<DnfNode>();

  // 2) Start ROS executor in its own thread
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  std::atomic<bool> ros_running{true};
  std::thread ros_thread([&](){
    exec.spin();
    ros_running = false;
  });

  // 3) Create and run your GUI app
  try {
    using namespace dnf_composer;

    const std::shared_ptr<dnf_composer::Simulation> previous_solution = std::make_shared<dnf_composer::Simulation>("packaging task control architecture");
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
            { "nf 5", "input" },
    });

    visualization->plot(
        PlotCommonParameters{
        PlotType::LINE_PLOT,
        PlotDimensions{ 0.0, 100, -20.0, 20, 1.0, 1.0},
        PlotAnnotations{ "target robot action field", "Spatial location", "Amplitude" } },
        LinePlotParameters{},
        { 
            { "nf 4", "activation" }, 
            { "nf 4", "input" },
    });

    app.addWindow<user_interface::MainWindow>();
    app.addWindow<imgui_kit::LogWindow>();
    app.addWindow<user_interface::FieldMetricsWindow>();
    app.addWindow<user_interface::ElementWindow>();
    app.addWindow<user_interface::SimulationWindow>();
    app.addWindow<user_interface::PlotControlWindow>();
    app.addWindow<user_interface::PlotsWindow>();
    app.addWindow<user_interface::NodeGraphWindow>();

    // Helper: apply amplitude to a GaussStimulus by element name (only in GUI thread)
    auto apply_presence = [&](const std::string& element_name, bool present) {
        auto base_elem = previous_solution->getElement(element_name);
        auto gs = std::dynamic_pointer_cast<dnf_composer::element::GaussStimulus>(base_elem);
        if (!gs) {
            log(dnf_composer::tools::logger::LogLevel::ERROR,
                "Element '" + element_name + "' is not a GaussStimulus.",
                dnf_composer::tools::logger::LogOutputMode::CONSOLE);
            return;
        }
        const auto p = gs->getParameters();
        const double amplitude = present ? 25.0 : 0.0;
        // keep width/position; only change amplitude
        gs->setParameters({ p.width, amplitude, p.position });
    };

    bool last_objs1 = false, last_objs2 = false, last_objl = false;


    auto classify_from_centroid = [](double c)->int {
        auto in = [&](double center){ return (c >= center - 2.0) && (c <= center + 2.0); };
        if (in(20.0)) return 1;
        if (in(50.0)) return 2;
        if (in(80.0)) return 3;
        return 0;
    };

    // returns current classification (0..3) or 0 if invalid
    auto compute_target_object = [&]() -> int {
        auto base = previous_solution->getElement("nf 4");
        auto nf4  = std::dynamic_pointer_cast<dnf_composer::element::NeuralField>(base);
        if (!nf4) {
            log(dnf_composer::tools::logger::LogLevel::ERROR,
                "Element 'nf 4' is not a NeuralField.",
                dnf_composer::tools::logger::LogOutputMode::CONSOLE);
            return 0;
        }

        const auto& bumps = nf4->getBumps();
        if (bumps.size() != 1) {
            // 0 bumps or >1 bumps -> 0 per your rule
            return 0;
        }

        const double centroid = bumps[0].centroid;  // <-- change if needed
        return classify_from_centroid(centroid);
    };
    
    int last_target_object = std::numeric_limits<int>::min();

    app.init();

   while (rclcpp::ok() && !app.hasGUIBeenClosed()) 
   {
        // read latest desired states from the node (atomics -> cheap)
        const bool cur_objs1 = node->objs1();
        const bool cur_objs2 = node->objs2();
        const bool cur_objl  = node->objl();

        // Apply only when the state changed
        if (cur_objs1 != last_objs1) {
            apply_presence("gs nf 1 20.000000", cur_objs1);
            last_objs1 = cur_objs1;
        }
        if (cur_objs2 != last_objs2) {
            apply_presence("gs nf 1 80.000000", cur_objs2);
            last_objs2 = cur_objs2;
        }
        if (cur_objl != last_objl) {
            apply_presence("gs nf 2 50.000000", cur_objl);
            last_objl = cur_objl;
        }

        int current = compute_target_object();
        if (current != last_target_object) {
            node->publish_target_object(current);
            last_target_object = current;
        }

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

  // 4) Shutdown sequence
  exec.cancel();
  if (ros_thread.joinable()) ros_thread.join();
  rclcpp::shutdown();
  return 0;
}