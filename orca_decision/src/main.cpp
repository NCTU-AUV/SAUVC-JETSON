#include <rclcpp/rclcpp.hpp>
#include "orca_decision/decision_node.hpp"
#include "orca_decision/behavior_tree_nodes.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>

void RegisterBehaviorTreeNodes(BT::BehaviorTreeFactory& factory, std::shared_ptr<DecisionContext> ctx) {
    factory.registerBuilder<SearchTarget>("SearchTarget", 
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<SearchTarget>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<ApproachTarget>("ApproachTarget", 
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<ApproachTarget>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<FinalAlignTarget>("FinalAlignTarget", 
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<FinalAlignTarget>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<BlindForward>("BlindForward", 
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<BlindForward>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<TurnToYaw>("TurnToYaw", 
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<TurnToYaw>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<AvoidObstacle>("AvoidObstacle", 
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<AvoidObstacle>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<SetCamera>("SetCamera", 
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<SetCamera>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<SetDepth>("SetDepth", 
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<SetDepth>(name, config);
            node->setContext(ctx);
            return node;
        });
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DecisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
