#include <rclcpp/rclcpp.hpp>
#include "orca_decision/decision_node.hpp"
#include "orca_decision/behavior_tree_nodes.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>

void RegisterBehaviorTreeNodes(BT::BehaviorTreeFactory& factory, std::shared_ptr<DecisionContext> ctx) {
    // ============================================================
    // Existing nodes
    // ============================================================
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

    // ============================================================
    // New nodes — Task 2: Target Acquisition
    // ============================================================
    factory.registerBuilder<MoveAboveTarget>("MoveAboveTarget",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<MoveAboveTarget>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<CheckBottomClear>("CheckBottomClear",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<CheckBottomClear>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<DropBall>("DropBall",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<DropBall>(name, config);
            node->setContext(ctx);
            return node;
        });

    // ============================================================
    // New nodes — Task 4: Communication & Localization
    // ============================================================
    factory.registerBuilder<WaitForFlareOrder>("WaitForFlareOrder",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<WaitForFlareOrder>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<BumpFlare>("BumpFlare",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<BumpFlare>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<SkipFlare>("SkipFlare",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<SkipFlare>(name, config);
            node->setContext(ctx);
            return node;
        });

    // ============================================================
    // New nodes — Task 3: Target Reacquisition
    // ============================================================
    factory.registerBuilder<GoToPose>("GoToPose",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<GoToPose>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<SearchBottomTarget>("SearchBottomTarget",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<SearchBottomTarget>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<SpiralSearchBottom>("SpiralSearchBottom",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<SpiralSearchBottom>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<ExtendArm>("ExtendArm",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<ExtendArm>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<GrabBall>("GrabBall",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<GrabBall>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<RetractArm>("RetractArm",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<RetractArm>(name, config);
            node->setContext(ctx);
            return node;
        });

    // ============================================================
    // New nodes — Mission Control
    // ============================================================
    factory.registerBuilder<WaitForStart>("WaitForStart",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<WaitForStart>(name, config);
            node->setContext(ctx);
            return node;
        });

    factory.registerBuilder<FinishMission>("FinishMission",
        [ctx](const std::string& name, const BT::NodeConfiguration& config) {
            auto node = std::make_unique<FinishMission>(name, config);
            node->setContext(ctx);
            return node;
        });

    // ============================================================
    // 新生實作區（方案 B）—— TODO(5)
    // ============================================================
    // 行為樹引擎只認得「註冊過」的節點名稱。在這裡把 StudentQualTask 註冊起來
    // 之前，trees.xml 裡不能出現 <StudentQualTask> —— BT.CPP 在**載入 XML 的
    // 當下**就會逐一檢查節點名稱，遇到不認識的就整份檔案載入失敗，連
    // FinalMission 一起陪葬（決策節點的 log 會是 "Failed to load BehaviorTree:
    // ... Node not recognized"）。所以 trees.xml 裡那棵 StudentMission 是先
    // 註解掉的，順序是：先註冊，再取消註解。
    //
    // 照著上面任何一個 registerBuilder<...> 的樣子寫一份就好。三件事不能少：
    //   1. 這個檔案最上面要 #include "orca_decision/student_qual_task.hpp"
    //   2. 註冊名稱要和 XML 上寫的標籤一字不差
    //   3. lambda 裡要呼叫 node->setContext(ctx)
    //
    // 第 3 點是最容易漏的：漏掉之後仍然編得過、也載入得了，但節點裡的 ctx_
    // 會是空的，你的 tick() 第一行就會走進黑板備援那條路。
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DecisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
