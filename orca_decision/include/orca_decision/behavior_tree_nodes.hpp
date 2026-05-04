#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "orca_decision/decision_context.hpp"

// Forward declaration of registration function
void RegisterBehaviorTreeNodes(BT::BehaviorTreeFactory& factory);

class SearchTarget : public BT::ActionNodeBase {
public:
    SearchTarget(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
};

class ApproachTarget : public BT::ActionNodeBase {
public:
    ApproachTarget(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    int stable_frames_ = 0;
    int lost_frames_ = 0;
};

class FinalAlignTarget : public BT::ActionNodeBase {
public:
    FinalAlignTarget(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time align_start_time_;
    bool aligning_ = false;
};

class BlindForward : public BT::ActionNodeBase {
public:
    BlindForward(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time start_time_;
    bool started_ = false;
    double target_yaw_ = 0.0;
};

class TurnToYaw : public BT::ActionNodeBase {
public:
    TurnToYaw(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
};

class AvoidObstacle : public BT::ActionNodeBase {
public:
    AvoidObstacle(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time last_avoid_time_;
    bool is_sticky_ = false;
};

class SetCamera : public BT::SyncActionNode {
public:
    SetCamera(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
};

class SetDepth : public BT::SyncActionNode {
public:
    SetDepth(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
};
