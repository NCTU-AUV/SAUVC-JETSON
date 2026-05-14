#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <Eigen/Dense>
#include "orca_decision/decision_context.hpp"

// Forward declaration of registration function
void RegisterBehaviorTreeNodes(BT::BehaviorTreeFactory& factory,
                               std::shared_ptr<DecisionContext> ctx);

// ============================================================
// Existing BT Nodes (unchanged)
// ============================================================

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

// ============================================================
// New BT Nodes — Task 2: Target Acquisition
// ============================================================

class MoveAboveTarget : public BT::ActionNodeBase {
public:
    MoveAboveTarget(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    int stable_frames_ = 0;
    rclcpp::Time start_time_;
    bool started_ = false;
    bool target_acquired_ = false;
    float filtered_error_x_ = 0.0f;
    float filtered_error_y_ = 0.0f;
};

class CheckBottomClear : public BT::ActionNodeBase {
public:
    CheckBottomClear(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time start_time_;
    bool started_ = false;
    int clear_frames_ = 0;
};

class DropBall : public BT::ActionNodeBase {
public:
    DropBall(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time phase_start_;
    bool started_ = false;
    int phase_ = 0;  // 0=stop, 1=release, 2=wait+record
};

// ============================================================
// New BT Nodes — Task 4: Communication & Localization
// ============================================================

class WaitForFlareOrder : public BT::ActionNodeBase {
public:
    WaitForFlareOrder(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time wait_start_time_;
    bool waiting_started_ = false;
};

class BumpFlare : public BT::ActionNodeBase {
public:
    BumpFlare(const std::string& name, const BT::NodeConfiguration& config);
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

class SkipFlare : public BT::SyncActionNode {
public:
    SkipFlare(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
};

// ============================================================
// New BT Nodes — Task 3: Target Reacquisition
// ============================================================

class GoToPose : public BT::ActionNodeBase {
public:
    GoToPose(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time start_time_;
    bool started_ = false;
};

class SearchBottomTarget : public BT::ActionNodeBase {
public:
    SearchBottomTarget(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time start_time_;
    bool started_ = false;
};

class SpiralSearchBottom : public BT::ActionNodeBase {
public:
    SpiralSearchBottom(const std::string& name, const BT::NodeConfiguration& config);
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

class ExtendArm : public BT::ActionNodeBase {
public:
    ExtendArm(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time start_time_;
    bool started_ = false;
};

class GrabBall : public BT::ActionNodeBase {
public:
    GrabBall(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time start_time_;
    bool started_ = false;
};

class RetractArm : public BT::ActionNodeBase {
public:
    RetractArm(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
    rclcpp::Time start_time_;
    bool started_ = false;
};

// ============================================================
// New BT Nodes — Mission Control
// ============================================================

class WaitForStart : public BT::ActionNodeBase {
public:
    WaitForStart(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void halt() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
};

class FinishMission : public BT::SyncActionNode {
public:
    FinishMission(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }
private:
    std::shared_ptr<DecisionContext> ctx_;
};
