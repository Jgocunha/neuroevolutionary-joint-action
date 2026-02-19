#include <memory>
#include <thread>
#include <atomic>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"

#include "kuka_lbr_iiwa14_marlab/msg/scene_object.hpp"
#include "kuka_lbr_iiwa14_marlab/msg/scene_objects.hpp"

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

    sub_scene_objects_ = create_subscription<kuka_lbr_iiwa14_marlab::msg::SceneObjects>(
      "scene_objects", qos,
      [this](kuka_lbr_iiwa14_marlab::msg::SceneObjects::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lk(objects_mtx_);
        latest_objects_.clear();
        latest_objects_.reserve(msg->objects.size());
        for (const auto &o : msg->objects)
        {
          char t = o.type.empty() ? ' ' : o.type[0];
          latest_objects_.push_back({t, o.position});
        }
      });

    sub_hand_position_ = create_subscription<std_msgs::msg::Float64>(
      "hand_position", qos,
      [this](std_msgs::msg::Float64::SharedPtr msg)
      {
        hand_position_.store(msg->data, std::memory_order_relaxed);
        hand_seen_.store(true, std::memory_order_relaxed);
      });

    pub_target_position_ = create_publisher<std_msgs::msg::Float64>("target_position", 200.0);

    double hz = declare_parameter<double>("target_position_rate_hz", 200.0);
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(1.0 / hz));

    timer_ = create_wall_timer(period, [this]{
      std_msgs::msg::Float64 msg;
      msg.data = target_position_.load(std::memory_order_relaxed);
      pub_target_position_->publish(msg);
    });
  }

  // Copy of current objects; called from GUI thread
  struct Obj {
    char   type{};
    double pos{};
    friend bool operator==(const Obj& a, const Obj& b) noexcept {
      // exact char match + tolerant double compare
      return a.type == b.type && std::abs(a.pos - b.pos) < 1e-9;
    }
  };
  std::vector<Obj> snapshot_objects() const
  {
    std::lock_guard<std::mutex> lk(objects_mtx_);
    return latest_objects_;
  }

  bool hand_seen() const { return hand_seen_.load(std::memory_order_relaxed); }
  double hand_position() const { return hand_position_.load(std::memory_order_relaxed); }
  void set_target_position(double v) { target_position_.store(v, std::memory_order_relaxed); }

private:
  rclcpp::Subscription<kuka_lbr_iiwa14_marlab::msg::SceneObjects>::SharedPtr sub_scene_objects_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_hand_position_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_target_position_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable std::mutex objects_mtx_;
  std::vector<Obj> latest_objects_;

  std::atomic<bool> hand_seen_{false};
  std::atomic<double> hand_position_{0.0};

  std::atomic<double> target_position_{200.0};
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

    const std::shared_ptr<Simulation> sim =
        std::make_shared<Simulation>("packaging task control architecture", 5.0);

    const SimulationFileManager sfm(sim,
      std::string(OUTPUT_DIRECTORY) + "/solution 120002 generation 121 species 1340 fitness 0.961601.json");
    sfm.loadElementsFromJson();

    auto visualization = std::make_shared<Visualization>(sim);
    const Application app{ sim, visualization };
    
    visualization->plot(
        PlotCommonParameters{
        PlotType::LINE_PLOT,
        PlotDimensions{ 0, 60, -20, 20, 1.0, 1.0},
        PlotAnnotations{ "Small object position input field", "Spatial location", "Amplitude" } },
        LinePlotParameters{},
        { 
            { "nf 1", "activation" }, 
            { "nf 1", "input" },
        }
    );

	visualization->plot(
        PlotCommonParameters{
        PlotType::LINE_PLOT,
        PlotDimensions{ 0, 60, -20, 20, 1.0, 1.0},
        PlotAnnotations{ "Large object position input field", "Spatial location", "Amplitude" } },
        LinePlotParameters{},
        { 
            { "nf 2", "activation" }, 
            { "nf 2", "input" },
        }
    );

    visualization->plot(
        PlotCommonParameters{
        PlotType::LINE_PLOT,
        PlotDimensions{ 0.0, 60, -20.0, 20, 1.0, 1.0},
        PlotAnnotations{ "Hand position input field", "Spatial location", "Amplitude" } },
        LinePlotParameters{},
        { 
            { "nf 3", "activation" }, 
            { "nf 3", "input" },
    });

    visualization->plot(
        PlotCommonParameters{
        PlotType::LINE_PLOT,
        PlotDimensions{ 0.0, 60, -20.0, 20, 1.0, 1.0},
        PlotAnnotations{ "Hidden field", "Spatial location", "Amplitude" } },
        LinePlotParameters{},
        { 
            { "nf 5", "activation" }, 
            { "gk cg 2 - 5 63", "output" },
            { "gk cg 3 - 5 64", "output" }, 
    });

    visualization->plot(
        PlotCommonParameters{
        PlotType::LINE_PLOT,
        PlotDimensions{ 0.0, 60, -20.0, 20, 1.0, 1.0},
        PlotAnnotations{ "Target robot action field", "Spatial location", "Amplitude" } },
        LinePlotParameters{},
        { 
            { "nf 4", "activation" }, 
            { "gk cg 1 - 4 34", "output" },
            { "gk cg 3 - 4 24", "output" },
            { "gk cg 5 - 4 62", "output" }, 
    });

    app.addWindow<user_interface::MainWindow>();
    //app.addWindow<imgui_kit::LogWindow>();
    //app.addWindow<user_interface::FieldMetricsWindow>();
    //app.addWindow<user_interface::ElementWindow>();
    //app.addWindow<user_interface::SimulationWindow>();
    //app.addWindow<user_interface::PlotControlWindow>();
    app.addWindow<user_interface::PlotsWindow>();
    //app.addWindow<user_interface::NodeGraphWindow>();

    // ---- Stimulus pools -----------------------------------------------------
    // We keep dynamic lists of GaussStimuli for nf1 (small) and nf2 (large).
    // On each scene_objects update, we (re)position the first K stimuli and
    // set amplitude=0 for any extras (so we avoid destroying elements).
    struct StimRef {
      std::string name; // element name in the graph
    };
    std::vector<StimRef> small_stims; // feed nf 1
    std::vector<StimRef> large_stims; // feed nf 2

    auto ensure_gauss_for_pool =
    [&](std::vector<StimRef>& pool, size_t needed, const std::string& nf_name, const std::string& prefix) {
        while (pool.size() < needed) {
        using namespace dnf_composer::element;
        const std::string elem_name = prefix + " " + std::to_string(pool.size());
        const GaussStimulusParameters stimulusParameters{3.0, 0.0, 30.0};
        const ElementDimensions& dimensions = ElementDimensions(60, 1.0);
        auto gaussStimulus = std::make_shared<GaussStimulus>(GaussStimulus{{elem_name, dimensions}, stimulusParameters});
        sim->addElement(gaussStimulus);
        sim->createInteraction(elem_name, "output", nf_name);
        pool.push_back({elem_name});
        }
    };


    auto set_gauss = [&](const std::string& elem_name, double width, double amp, double pos)
    {
      auto base = sim->getElement(elem_name);
      auto gs = std::dynamic_pointer_cast<dnf_composer::element::GaussStimulus>(base);
      if (!gs)
      {
        log(dnf_composer::tools::logger::LogLevel::ERROR,
            "Element '" + elem_name + "' is not a GaussStimulus.",
            dnf_composer::tools::logger::LogOutputMode::CONSOLE);
        return;
      }
      gs->setParameters({ width, amp, pos });
    };

    // Dedicated hand stimulus into nf 3 (we keep exactly one)
    const std::string hand_gs_name = "gs hand";
    {
      // Create if missing
      if (!std::dynamic_pointer_cast<dnf_composer::element::GaussStimulus>(sim->getElement(hand_gs_name)))
      {
        using namespace dnf_composer::element;
        const GaussStimulusParameters stimulusParameters{/*width*/3.0, /*amp*/0.0, /*pos*/30.0};
        const ElementDimensions& dimensions = ElementDimensions(60, 1.0);
        const auto gaussStimulus = std::make_shared<GaussStimulus>(GaussStimulus{ { hand_gs_name, dimensions }, stimulusParameters });
		    sim->addElement(gaussStimulus);
        sim->createInteraction(hand_gs_name, "output", "nf 3");
      }
    }

    // NF4 centroid → target_position
    auto compute_target_position = [&]() -> double
    {
      auto base = sim->getElement("nf 4");
      auto nf4  = std::dynamic_pointer_cast<dnf_composer::element::NeuralField>(base);
      if (!nf4)
      {
        log(dnf_composer::tools::logger::LogLevel::ERROR,
            "Element 'nf 4' is not a NeuralField.",
            dnf_composer::tools::logger::LogOutputMode::CONSOLE);
        return 100.0;
      }
      const auto& bumps = nf4->getBumps();
      if (bumps.size() != 1) return 100.0;
      return bumps[0].centroid;
    };

    app.init();

    // Cache to detect when scene objects changed
    std::vector<DnfNode::Obj> last_objs;

    while (rclcpp::ok() && !app.hasGUIBeenClosed())
    {
      // --- 1) Scene objects → nf1/nf2 stimuli
      auto cur_objs = node->snapshot_objects();
      if (cur_objs != last_objs)
      {
        // Partition into small vs large while preserving order
        std::vector<double> small_positions, large_positions;
        small_positions.reserve(cur_objs.size());
        large_positions.reserve(cur_objs.size());
        for (const auto& o : cur_objs)
        {
          if (o.type == 's' || o.type == 'S') small_positions.push_back(o.pos);
          else if (o.type == 'l' || o.type == 'L') large_positions.push_back(o.pos);
        }

        // Ensure enough stimuli in each pool
        ensure_gauss_for_pool(small_stims, small_positions.size(), "nf 1", "gs nf1 s");
        ensure_gauss_for_pool(large_stims, large_positions.size(), "nf 2", "gs nf2 l");

        // Activate & place the first K stimuli
        for (size_t i = 0; i < small_positions.size(); ++i)
        {
            if (!(small_positions[i] < 0.0 || small_positions[i] > 60.0))
                set_gauss(small_stims[i].name, /*w*/3.0, /*amp*/6.0, small_positions[i]);
            else 
                set_gauss(small_stims[i].name, /*w*/3.0, /*amp*/0.0, /*pos*/30.0);
        }
        for (size_t i = 0; i < large_positions.size(); ++i)
        {
            if (!(large_positions[i] < 0.0 || large_positions[i] > 60.0))
                set_gauss(large_stims[i].name, /*w*/3.0, /*amp*/6.0, large_positions[i]);
            else 
                set_gauss(large_stims[i].name, /*w*/3.0, /*amp*/0.0, /*pos*/30.0);
        }

        // Mute any extra (previously-created) stimuli
        for (size_t i = small_positions.size(); i < small_stims.size(); ++i)
          set_gauss(small_stims[i].name, /*w*/3.0, /*amp*/0.0, /*pos*/30.0);
        for (size_t i = large_positions.size(); i < large_stims.size(); ++i)
          set_gauss(large_stims[i].name, /*w*/3.0, /*amp*/0.0, /*pos*/30.0);

        last_objs = std::move(cur_objs);
      }

      // --- 2) Hand position → nf3 stimulus
      if (node->hand_seen())
      {
        const double hp = node->hand_position();
        if (!(hp < 0.0 || hp > 60.0))
            set_gauss(hand_gs_name, /*w*/3.0, /*amp*/6.0, hp);
        else 
            set_gauss(hand_gs_name, /*w*/3.0, /*amp*/0.0, /*pos*/30.0);
      }

      // --- 3) Read nf4 and publish target_position (via timer)
      const double target = compute_target_position();
      node->set_target_position(target);

      // --- 4) Step the app (render + sim tick)
      app.step();
    }

    app.close();
  }
  catch (const dnf_composer::Exception& ex)
  {
    const std::string msg = std::string("Exception: ") + ex.what() +
      " ErrorCode: " + std::to_string(static_cast<int>(ex.getErrorCode())) + ". ";
    log(dnf_composer::tools::logger::LogLevel::FATAL, msg,
        dnf_composer::tools::logger::LogOutputMode::CONSOLE);
  }
  catch (const std::exception& ex)
  {
    log(dnf_composer::tools::logger::LogLevel::FATAL,
        "Exception caught: " + std::string(ex.what()) + ". ",
        dnf_composer::tools::logger::LogOutputMode::CONSOLE);
  }
  catch (...)
  {
    log(dnf_composer::tools::logger::LogLevel::FATAL,
        "Unknown exception occurred. ",
        dnf_composer::tools::logger::LogOutputMode::CONSOLE);
  }

  exec.cancel();
  if (ros_thread.joinable()) ros_thread.join();
  rclcpp::shutdown();
  return 0;
}