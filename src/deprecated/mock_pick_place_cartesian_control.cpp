#include <map>
#include <memory>
#include <mutex>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

using namespace std::chrono_literals;

class MockPickPlace 
  : public rclcpp::Node
{
public:
  MockPickPlace(const rclcpp::NodeOptions & options)
    : Node("mock_pick_place", options) 
{
    //declare_parameter("robot_name", "lbr");
    sub_ = create_subscription<std_msgs::msg::Int32>(
      "target_object", 10,
      std::bind(&MockPickPlace::targetCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Constructed. Call init() before spin().");
  }

  void init(const rclcpp::Node::SharedPtr &self)
  {
    robot_name_ = get_parameter("robot_name").as_string();

    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      self,
      moveit::planning_interface::MoveGroupInterface::Options(
        "arm",
        "robot_description",
        robot_name_));

    move_group_->setStartStateToCurrentState();

    targets_ = {
    //   {1,{ make_pose(9.1499, 5.2540, 1.1247), make_pose(9.14, 5.2540, 1.0247), make_pose(9.14, 4.2540, 1.1247) }},
    //   {2,{ make_pose(9.1499, 4.9540, 1.1197), make_pose(9.1499, 4.9540, 1.0197), make_pose(9.14, 4.2540, 1.1247) }},
    //   {3,{ make_pose(9.1499, 4.6540, 1.1247), make_pose(9.1499, 4.6540, 1.0247), make_pose(9.14, 4.2540, 1.1247) }},
    {1,{ make_pose(0.4, -0.4, 0.8), make_pose(0.4, -0.4, 0.7), make_pose(0.4, -0.4, 0.6) }},
    {2,{ make_pose(0.4, 0.1, 0.8), make_pose(0.4, 0.2, 0.8), make_pose(0.4, 0.3, 0.8) }},
    {3,{ make_pose(0.5, 0.3, 0.8), make_pose(0.6, 0.3, 0.8), make_pose(0.7, 0.3, 0.8) }},
    }; // pre-grasp, grasp. place 

    RCLCPP_INFO(get_logger(), "MoveGroup initialized; ready.");
  }

private:
  struct Triplet { geometry_msgs::msg::Pose pre, grasp, place; };

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr           sub_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::map<int,Triplet>                                         targets_;
  std::string                                                   robot_name_;
  std::mutex                                                    mutex_;
  std::set<int>                                                 completed_;

  static geometry_msgs::msg::Pose make_pose(double x,double y,double z)
  {
    geometry_msgs::msg::Pose p;
    p.orientation.w = 1.0;
    p.position.x    = x;
    p.position.y    = y;
    p.position.z    = z;
    return p;
  }

  void targetCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    int id = msg->data;
    if (!targets_.count(id)) {
      RCLCPP_WARN(get_logger(),"Ignoring invalid %d",id);
      return;
    }
    if (completed_.count(id)) {
      RCLCPP_INFO(get_logger(),"Already did %d",id);
      return;
    }
    RCLCPP_INFO(get_logger(),"Pick & place %d",id);
    doPickPlace(id);
    completed_.insert(id);
  }

  void doPickPlace(int id)
  {
    auto &t = targets_.at(id);
    planAndExec(t.pre,   "pre-grasp");
    planCartesianAndExec(t.pre, t.pre,   "pre-grasp");
    RCLCPP_INFO(get_logger(), "Executing pre-grasp");
    rclcpp::sleep_for(500ms);
    planCartesianAndExec(t.pre, t.grasp, "grasp");
    RCLCPP_INFO(get_logger(), "Executing grasp");
    //RCLCPP_INFO(get_logger(),"-- mock close");
    rclcpp::sleep_for(500ms);
    //planCartesianAndExec(t.grasp, t.pre,   "retract");
    planCartesianAndExec(t.grasp, t.place, "place");
    RCLCPP_INFO(get_logger(), "Executing place");
    //RCLCPP_INFO(get_logger(),"-- mock open");
    rclcpp::sleep_for(500ms);
  }

  bool planAndExec(const geometry_msgs::msg::Pose &pose,const std::string &lbl)
  {
    //move_group_->setPoseReferenceFrame("world");
    move_group_->setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan)==moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(get_logger(),"Exec %s",lbl.c_str());
      move_group_->execute(plan);
      return true;
    }
    RCLCPP_ERROR(get_logger(),"Plan %s failed",lbl.c_str());
    return false;
  }

  bool planCartesianAndExec(const geometry_msgs::msg::Pose &current,
    const geometry_msgs::msg::Pose &target, const std::string &lbl)
  {
    //move_group_->setStartStateToCurrentState();

    // 1) grab the current end-effector pose
    auto stamped = move_group_->getCurrentPose(); 
    geometry_msgs::msg::Pose start_pose = stamped.pose;

     // print current pose
    RCLCPP_INFO(get_logger(),"Current user-defined pose: %f %f %f",
                 current.position.x,
                 current.position.y,
                 current.position.z);

    // print current pose
    RCLCPP_INFO(get_logger(),"Current /joint_states pose: %f %f %f",
                 start_pose.position.x,
                 start_pose.position.y,
                 start_pose.position.z);

    // 2) build waypoints
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(current);
    waypoints.push_back(target);

    // 3) ensure MoveIt uses the very latest joint state
    move_group_->setStartStateToCurrentState();

    // 4) compute Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_->computeCartesianPath(
        waypoints,
        /*eef_step*/        0.01,
        /*jump_threshold*/  0.0,
        trajectory);

    if (fraction < 0.99) {
        RCLCPP_ERROR(get_logger(),
        "Cartesian plan %s only %.1f%% complete", lbl.c_str(), fraction * 100.0);
        return false;
    }

    // 5) execute
    moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
    cart_plan.trajectory_ = trajectory;
    RCLCPP_INFO(get_logger(), "Exec %s (Cartesian)", lbl.c_str());
    move_group_->execute(cart_plan);
    return true;
    }

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // construct node
  rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<MockPickPlace>(options);
  // now init MoveGroup with the same shared_ptr
  node->init(node);
  // spin
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
