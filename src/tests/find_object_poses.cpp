#include <map>
#include <memory>
#include <mutex>
#include <set>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

using namespace std::chrono_literals;

class CartesianPathPlanningTest : public rclcpp::Node {
public:
  CartesianPathPlanningTest(const rclcpp::NodeOptions & options)
  : Node("find_object_poses_test", options) {
    timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  }

  void init(const rclcpp::Node::SharedPtr & self) {
    std::string robot_name = get_parameter("robot_name").as_string();

    auto opts = moveit::planning_interface::MoveGroupInterface::Options(
        "arm", "robot_description", robot_name);

    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(self, opts);

    move_group_->setStartStateToCurrentState();
    move_group_->setGoalPositionTolerance(2e-3);
    move_group_->setGoalOrientationTolerance(5e-2);

    // Slow down so controllers don’t induce flips from aggressive time parameterization
    move_group_->setMaxVelocityScalingFactor(0.2);
    move_group_->setMaxAccelerationScalingFactor(0.2);

    moveit_msgs::msg::Constraints path_constraints;
    // Keep A2 in a safe band
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = "lbr_A2";
    jc.position = 0.0;                // center of band (rad)
    jc.tolerance_above = 115.0 * M_PI/180.0;
    jc.tolerance_below = 115.0 * M_PI/180.0;
    jc.weight = 1.0;
    path_constraints.joint_constraints.push_back(jc);
    move_group_->setPathConstraints(path_constraints);

    RCLCPP_INFO(get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(get_logger(), "MoveGroup initialized; ready.");

    step_timer_ = rclcpp::create_timer(
        this, this->get_clock(), 7s,
        std::bind(&CartesianPathPlanningTest::step, this),
        timer_group_);
  }

private:
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::TimerBase::SharedPtr step_timer_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;
  int stage_{0};

  static geometry_msgs::msg::Pose make_pose(double px,double py,double pz,
                                            double ox,double oy,double oz,double ow) {
    geometry_msgs::msg::Pose p;
    p.position.x = px; p.position.y = py; p.position.z = pz;
    p.orientation.x = ox; p.orientation.y = oy; p.orientation.z = oz; p.orientation.w = ow;
    return p;
  }

  void do_cartesian(const geometry_msgs::msg::Pose &target, const std::string &lbl) {
    // make sure monitor has a fresh state
    auto state = move_group_->getCurrentState(0.5);
    if (!state) RCLCPP_WARN(get_logger(), "No joint state within 0.5s");

    move_group_->setStartStateToCurrentState();

    auto start_pose = move_group_->getCurrentPose().pose;
    RCLCPP_INFO(get_logger(),"Current pose: %f %f %f %f %f %f %f",
                start_pose.position.x, start_pose.position.y, start_pose.position.z,
                start_pose.orientation.x, start_pose.orientation.y,
                start_pose.orientation.z, start_pose.orientation.w);


    auto new_target = target;
    new_target.orientation.x = start_pose.orientation.x;
    new_target.orientation.y = start_pose.orientation.y;
    new_target.orientation.z = start_pose.orientation.z;
    new_target.orientation.w = start_pose.orientation.w;

    std::vector<geometry_msgs::msg::Pose> waypoints{start_pose, new_target};

    moveit_msgs::msg::RobotTrajectory traj;
    double fraction = move_group_->computeCartesianPath(
        waypoints, /*eef_step*/0.005, /*jump_threshold*/0.0, traj, /*avoid_collisions=*/true);
        // Use a nonzero jump threshold so MoveIt prunes paths that require joint “teleports”

    if (fraction < 0.50) {
      RCLCPP_ERROR(get_logger(), "Cartesian plan %s only %.1f%% complete",
                   lbl.c_str(), fraction*100.0);
      return;
    }
    RCLCPP_INFO(get_logger(), "Cartesian plan %s %.1f%% complete",
                   lbl.c_str(), fraction*100.0);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = traj;

     RCLCPP_INFO(get_logger(), "Exec %s (Cartesian)", lbl.c_str());

    robot_trajectory::RobotTrajectory rt(
        move_group_->getCurrentState()->getRobotModel(),
        move_group_->getName());
    rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), traj);

    // Time parameterization
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    bool ok = totg.computeTimeStamps(rt, /*vel_scale=*/0.30, /*acc_scale=*/0.30);
    if (!ok) {
      RCLCPP_ERROR(get_logger(), "TOTG failed");
      return;
    }
    rt.getRobotTrajectoryMsg(traj);
    plan.trajectory_ = traj;

    // Execute ONCE
    auto ec = move_group_->execute(plan);
    if (ec != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Execute failed: %d", ec.val);
      return;
    }

    (void)move_group_->getCurrentState(0.5);
    move_group_->setStartStateToCurrentState();
  }

  void step() {
    switch (stage_) {
      // table centre position position(0.662, -0.005, 0.951) orientation(0.847, -0.120, 0.509, 0.098)
      // 0.754, -0.034, 0.655, -0.028

      // very nice pose for pick up object centre
      // Pose: position(0.581, 0.060, 1.133) orientation(0.866, 0.030, 0.500, 0.014)

      // original 25 cm each object 100cm dimension
      //case 0: do_cartesian(make_pose(0.7506, -0.0668, 1.033, 0.761, -0.203, 0.611, 0.079), "over the table centre position"); break;
      
      // case 0: do_cartesian(make_pose(0.7506,  0.3132, 1.033, 0.761, -0.203, 0.611, 0.079), "over left-most object pre-pick"); break;
      // case 1: do_cartesian(make_pose(0.7506,  0.3132, 0.983, 0.761, -0.203, 0.611, 0.079), "over left-most object pick"); break;
      // case 2: do_cartesian(make_pose(0.7506,  0.3132, 1.033, 0.761, -0.203, 0.611, 0.079), "over left-most object pre-pick"); break;

      // case 3: do_cartesian(make_pose(0.7506,  0.0632, 1.033, 0.761, -0.203, 0.611, 0.079), "over centre object pre-pick"); break;
      // case 4: do_cartesian(make_pose(0.7506,  0.0632, 0.983, 0.761, -0.203, 0.611, 0.079), "over centre object pick"); break;
      // case 5: do_cartesian(make_pose(0.7506,  0.0632, 1.033, 0.761, -0.203, 0.611, 0.079), "over centre object pre-pick"); break;

      // case 6: do_cartesian(make_pose(0.7506, -0.1868, 1.033, 0.761, -0.203, 0.611, 0.079), "over right-most object pre-pick"); break;
      // case 7: do_cartesian(make_pose(0.7506, -0.1868, 0.983, 0.761, -0.203, 0.611, 0.079), "over right-most object pick"); break;
      // case 8: do_cartesian(make_pose(0.7506, -0.1868, 1.033, 0.761, -0.203, 0.611, 0.079), "over right-most object pre-pick"); break;

      // new: 20 cm each object 60cm dimension left arm

      //case 0: do_cartesian(make_pose(0.7006, 0.3132, 1.083, 0.761, -0.203, 0.611, 0.079), "over left-most object pre-pick"); break;
      //case 1: do_cartesian(make_pose(0.7506, 0.3132, 1.010, 0.761, -0.203, 0.611, 0.079), "over left-most object pick"); break;
      //case 2: do_cartesian(make_pose(0.7006, 0.3132, 1.083, 0.761, -0.203, 0.611, 0.079), "over left-most object pre-pick"); break;

      //case 3: do_cartesian(make_pose(0.7006, 0.1132, 1.083, 0.761, -0.203, 0.611, 0.079), "over centre object pre-pick"); break;
      //case 4: do_cartesian(make_pose(0.7506, 0.1132, 1.010, 0.761, -0.203, 0.611, 0.079), "over centre object pick"); break;
      //case 5: do_cartesian(make_pose(0.7006, 0.1132, 1.083, 0.761, -0.203, 0.611, 0.079), "over centre object pre-pick"); break;

      //case 6: do_cartesian(make_pose(0.7006, -0.0858, 1.083, 0.761, -0.203, 0.611, 0.079), "over right-most object pre-pick"); break;
      //case 7: do_cartesian(make_pose(0.7506, -0.0858, 1.010, 0.761, -0.203, 0.611, 0.079), "over right-most object pick"); break;
      //case 8: do_cartesian(make_pose(0.7006, -0.0858, 1.083, 0.761, -0.203, 0.611, 0.079), "over right-most object pre-pick"); break;

      // case  9: do_cartesian(make_pose(0.7006, 0.586, 1.103, 0.761, -0.203, 0.611, 0.079), "pre-place"); break;
      // case 10: do_cartesian(make_pose(0.7506, 0.586, 1.060, 0.761, -0.203, 0.611, 0.079), "place"); break;
      // case 11: do_cartesian(make_pose(0.7006, 0.586, 1.103, 0.761, -0.203, 0.611, 0.079), "pre-place"); break;

      // 60cm dimension right arm (relative to the robot)
      case 0: do_cartesian(make_pose(0.7006, -0.7332, 1.103, 0.761, -0.203, 0.611, 0.079), "table limit right side"); break;

      case 1: do_cartesian(make_pose(0.7006, -0.6513, 1.103, 0.761, -0.203, 0.611, 0.079), "pre-place"); break;
      case 2: do_cartesian(make_pose(0.7506, -0.6513, 1.060, 0.761, -0.203, 0.611, 0.079), "place"); break;
      case 3: do_cartesian(make_pose(0.7006, -0.6513, 1.103, 0.761, -0.203, 0.611, 0.079), "pre-place"); break;

      case 4: do_cartesian(make_pose(0.7506,  0.0487, 1.033, 0.761, -0.203, 0.611, 0.079), "over left-most object pre-pick"); break;
      case 5: do_cartesian(make_pose(0.7506,  0.0487, 0.983, 0.761, -0.203, 0.611, 0.079), "over left-most object pick"); break;
      case 6: do_cartesian(make_pose(0.7506,  0.0487, 1.033, 0.761, -0.203, 0.611, 0.079), "over left-most object pre-pick"); break;

      case 7: do_cartesian(make_pose(0.7506,  -0.1513, 1.033, 0.761, -0.203, 0.611, 0.079), "over centre object pre-pick"); break;
      case 8: do_cartesian(make_pose(0.7506,  -0.1513, 0.983, 0.761, -0.203, 0.611, 0.079), "over centre object pick"); break;
      case 9: do_cartesian(make_pose(0.7506,  -0.1513, 1.033, 0.761, -0.203, 0.611, 0.079), "over centre object pre-pick"); break;

      case 10: do_cartesian(make_pose(0.7506, -0.3513, 1.033, 0.761, -0.203, 0.611, 0.079), "over right-most object pre-pick"); break;
      case 11: do_cartesian(make_pose(0.7506, -0.3513, 0.983, 0.761, -0.203, 0.611, 0.079), "over right-most object pick"); break;
      case 12: do_cartesian(make_pose(0.7506, -0.3513, 1.033, 0.761, -0.203, 0.611, 0.079), "over right-most object pre-pick"); break;

      default: step_timer_->cancel(); return;
    }
    stage_++;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<CartesianPathPlanningTest>(options);
  node->init(node);

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
