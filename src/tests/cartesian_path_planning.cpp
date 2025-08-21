#include <map>
#include <memory>
#include <mutex>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

using namespace std::chrono_literals;

class CartesianPathPlanningTest 
  : public rclcpp::Node
{
public:
  CartesianPathPlanningTest(const rclcpp::NodeOptions & options)
    : Node("cartesian_path_planning_test", options) 
{}

  void init(const rclcpp::Node::SharedPtr &self)
  {
    std::string robot_name = get_parameter("robot_name").as_string();

    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      self,
      moveit::planning_interface::MoveGroupInterface::Options(
        "arm",
        "robot_description",
        robot_name));

    move_group_->setStartStateToCurrentState();
    //move_group_->setPoseReferenceFrame("world");
    move_group_->setGoalPositionTolerance(1e-3);     // 1 mm (or 2–3 mm to start)
    move_group_->setGoalOrientationTolerance(1e-2);  // ~0.6 deg (or a bit larger)
    auto state = move_group_->getCurrentState(2.0);  // wait up to 2s for first joint state
    if (!state) {
      RCLCPP_WARN(get_logger(), "No joint state received within 2s");
    }

    RCLCPP_INFO(get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(get_logger(), "MoveGroup initialized; ready.");

    step_timer_ = create_wall_timer(2000ms, std::bind(&CartesianPathPlanningTest::step, this));
  }

private:
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::TimerBase::SharedPtr step_timer_;
  int stage_{0};

  static geometry_msgs::msg::Pose make_pose(double px,double py, double pz, 
                                            double ox, double oy, double oz, double ow)
  {
    geometry_msgs::msg::Pose p;
    p.position.x    = px;
    p.position.y    = py;
    p.position.z    = pz;
    p.orientation.x = ox;
    p.orientation.y = oy;
    p.orientation.z = oz;
    p.orientation.w = ow;
    return p;
  }

  void do_cartesian(const geometry_msgs::msg::Pose &target, 
                  const std::string &lbl)
  {
    auto state = move_group_->getCurrentState(2.0);  // wait up to 2s for first joint state
    if (!state) {
      RCLCPP_WARN(get_logger(), "No joint state received within 2s");
    }
    if (state) {
      std::vector<double> q;
      state->copyJointGroupPositions(move_group_->getName(), q);
      RCLCPP_INFO(get_logger(), "q0=%.3f q1=%.3f ...", q[0], q[1]);
    }
    move_group_->setStartStateToCurrentState();

    auto stamped = move_group_->getCurrentPose(); 
    geometry_msgs::msg::Pose start_pose = stamped.pose;

    RCLCPP_INFO(get_logger(),"Current pose: %f %f %f %f %f %f %f",
                  start_pose.position.x,
                  start_pose.position.y,
                  start_pose.position.z,
                  start_pose.orientation.x,
                  start_pose.orientation.y,
                  start_pose.orientation.z,
                  start_pose.orientation.w);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(start_pose);
    waypoints.push_back(target);

    moveit_msgs::msg::RobotTrajectory trajectory;    
    double fraction = move_group_->computeCartesianPath(
        waypoints,
        /*eef_step*/        0.005,
        /*jump_threshold*/  0.0,
        trajectory);

    if (fraction < 0.99) {
        RCLCPP_ERROR(get_logger(),
        "Cartesian plan %s only %.1f%% complete", lbl.c_str(), fraction * 100.0);
        return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
    cart_plan.trajectory_ = trajectory;
    RCLCPP_INFO(get_logger(), "Exec %s (Cartesian)", lbl.c_str());
    move_group_->execute(cart_plan);

    // give the monitor time to receive the new joint state
    state = move_group_->getCurrentState(2.0);  // wait up to 2s for first joint state
    if (!state) {
      RCLCPP_WARN(get_logger(), "No joint state received within 2s");
    }
    if (state) {
      std::vector<double> q;
      state->copyJointGroupPositions(move_group_->getName(), q);
      RCLCPP_INFO(get_logger(), "q0=%.3f q1=%.3f ...", q[0], q[1]);
    }
  }

    void step() 
    {
      const double eef_length = 0.199;
      switch (stage_) 
      {
        case 0: do_cartesian(make_pose(9.1499 + eef_length, 5.51667, 1.2247, 0.031735, -0.701920, 0.032244, 0.710818), "pre_pick_1"); break;
        case 1: do_cartesian(make_pose(9.1499 + eef_length, 5.51667, 1.1247, 0.031735, -0.701920, 0.032244, 0.710818), "pick_1"); break;
        case 2: do_cartesian(make_pose(9.1499 + eef_length, 5.51667, 1.2247, 0.031735, -0.701920, 0.032244, 0.710818), "post_pick_1"); break;

        case 3: do_cartesian(make_pose(9.1499 + eef_length, 5.18334, 1.2247, 0.031735, -0.701920, 0.032244, 0.710818), "pre_pick_2"); break;
        case 4: do_cartesian(make_pose(9.1499 + eef_length, 5.18334, 1.1247, 0.031735, -0.701920, 0.032244, 0.710818), "pick_2"); break;
        case 5: do_cartesian(make_pose(9.1499 + eef_length, 5.18334, 1.2247, 0.031735, -0.701920, 0.032244, 0.710818), "post_pick_2"); break;

        case 6: do_cartesian(make_pose(9.1499 + eef_length, 4.85000, 1.2247, 0.031735, -0.701920, 0.032244, 0.710818), "pre_pick_3"); break;
        case 7: do_cartesian(make_pose(9.1499 + eef_length, 4.85000, 1.1247, 0.031735, -0.701920, 0.032244, 0.710818), "pick_3"); break;
        case 8: do_cartesian(make_pose(9.1499 + eef_length, 4.85000, 1.2247, 0.031735, -0.701920, 0.032244, 0.710818), "post_pick_3"); break;

        case 9:   do_cartesian(make_pose(9.1499 + eef_length, 5.850, 1.2247, 0.031735, -0.701920, 0.032244, 0.710818), "pre_place"); break;
        case 10:  do_cartesian(make_pose(9.1499 + eef_length, 5.850, 1.1247, 0.031735, -0.701920, 0.032244, 0.710818), "place"); break;
        case 11:  do_cartesian(make_pose(9.1499 + eef_length, 5.850, 1.2247, 0.031735, -0.701920, 0.032244, 0.710818), "post_place"); break;

        default: step_timer_->cancel(); return;
      }
      stage_++;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<class CartesianPathPlanningTest>(options);

    node->init(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}