// #region server
#include "my_package_interfaces/action/example_simple_skill.hpp"
#include "auto_apms_util/action_wrapper.hpp"

namespace my_namespace 
{
using SimpleSkillActionType = my_package_interfaces::action::ExampleSimpleSkill;

class SimpleSkillServer : public auto_apms_util::ActionWrapper<SimpleSkillActionType>
{
public:
  SimpleSkillServer(const rclcpp::NodeOptions & options) 
  : ActionWrapper("simple_skill", options) {}

  // Callback invoked when a goal arrives
  bool onGoalRequest(std::shared_ptr<const Goal> goal_ptr) override final
  {
    index_ = 1;
    start_ = node_ptr_->now();
    return true;
  }

  // Callback invoked asynchronously by the internal execution routine 
  Status executeGoal(
    std::shared_ptr<const Goal> goal_ptr, 
    std::shared_ptr<Feedback> feedback_ptr,
    std::shared_ptr<Result> result_ptr) override final
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "#%i - %s", index_++, goal_ptr->msg.c_str());
    if (index_ <= goal_ptr->n_times) {
      return Status::RUNNING;
    }
    result_ptr->time_required = (node_ptr_->now() - start_).to_chrono<std::chrono::duration<double>>().count();
    return Status::SUCCESS;
  }

private:
  uint8_t index_;
  rclcpp::Time start_;
};
}  // namespace my_namespace

// Register the skill as a ROS 2 component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_namespace::SimpleSkillServer)
// #endregion server

// #region client
#include "my_package_interfaces/action/example_simple_skill.hpp"
#include "auto_apms_behavior_tree/node.hpp"

namespace my_namespace
{
using SimpleSkillActionType = my_package_interfaces::action::ExampleSimpleSkill;

class SimpleSkillClient : public auto_apms_behavior_tree::core::RosActionNode<SimpleSkillActionType>
{
public:
  using RosActionNode::RosActionNode;

  // We must define data ports to accept arguments
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("msg"), 
            BT::InputPort<uint8_t>("n_times")};
  }
 
  // Callback invoked to specify the action goal
  bool setGoal(Goal & goal) override final
  {
    RCLCPP_INFO(logger_, "--- Set goal ---");
    goal.msg = getInput<std::string>("msg").value();
    goal.n_times = getInput<uint8_t>("n_times").value();
    return true;
  }
  
  // Callback invoked when the action is finished
  BT::NodeStatus onResultReceived(const WrappedResult & result) override final
  {
    RCLCPP_INFO(logger_, "--- Result received ---");
    RCLCPP_INFO(logger_, "Time elapsed: %f", result.result->time_required);
    return RosActionNode::onResultReceived(result);
  }
};
}  // namespace my_namespace

// Make the node discoverable for the class loader
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(my_namespace::SimpleSkillClient)
// #endregion client

// #region build_handler
// This also includes all standard node models 
// (under namespace auto_apms_behavior_tree::model)
#include "auto_apms_behavior_tree/build_handler.hpp"

// Include our custom node models 
// (under namespace my_package::model)
#include "my_package/simple_skill_nodes.hpp"

namespace my_namespace
{

class SimpleSkillBuildHandler : public auto_apms_behavior_tree::TreeBuildHandler
{
public:
  using TreeBuildHandler::TreeBuildHandler;

  TreeDocument::TreeElement buildTree(
    TreeDocument & doc, 
    TreeBlackboard & bb) override final
  {
    // Create an empty tree element
    TreeDocument::TreeElement tree = doc.newTree("SimpleSkillDemo").makeRoot();

    // Alias for the standard node model namespace
    namespace standard_model = auto_apms_behavior_tree::model;

    // Insert the root sequence as the first element to the tree
    TreeDocument::NodeElement sequence = tree.insertNode<standard_model::Sequence>();

    // Insert the HasParameter node for the variable msg
    sequence.insertNode<standard_model::ForceSuccess>()
      .insertNode<model::HasParameter>()
      .set_parameter("bb.msg")
      .setConditionalScript(BT::PostCond::ON_SUCCESS, "msg := @msg")
      .setConditionalScript(BT::PostCond::ON_FAILURE, "msg := 'No blackboard parameter'");

    // Insert the HasParameter node for the variable n_times
    sequence.insertNode<standard_model::ForceSuccess>()
      .insertNode<model::HasParameter>()
      .set_parameter("bb.n_times")
      .setConditionalScript(BT::PostCond::ON_SUCCESS, "n_times := @n_times")
      .setConditionalScript(BT::PostCond::ON_FAILURE, "n_times := 1");

    // Insert the SimpleSkillActionNode node that prints msg exactly n_times times
    sequence.insertNode<model::SimpleSkillActionNode>()
      .set_n_times("{n_times}")
      .set_msg("{msg}");

    // Insert the SimpleSkillActionNode node that prints the last message
    sequence.insertNode<model::SimpleSkillActionNode>()
      .set_n_times(1)
      .set_msg("Last message");

    // Return the tree to be executed
    return tree;
  }
};

}  // namespace my_namespace

// Make the build handler discoverable for the class loader
AUTO_APMS_BEHAVIOR_TREE_REGISTER_BUILD_HANDLER(my_namespace::SimpleSkillBuildHandler)
// #endregion build_handler