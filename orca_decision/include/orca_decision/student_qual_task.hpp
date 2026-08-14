#pragma once

// 新生實作用的行為樹節點（資格賽任務，方案 B）。
//
// 正式節點的宣告全部集中在 behavior_tree_nodes.hpp。這一個刻意拆成獨立檔案：
// 每個人只改自己的檔案，不會在同一份 header 上互相衝突，寫壞了也不影響
// 比賽用的那棵樹。實作完成、要進正式流程時再併回 behavior_tree_nodes.hpp。
//
// 對照組是 BlindForward（src/behavior_tree_nodes/blind_forward.cpp），
// 它做的事情跟你要寫的節點高度重疊，卡住時先去看它。

#include <behaviortree_cpp_v3/action_node.h>

#include "orca_decision/decision_context.hpp"

// ActionNodeBase：需要跨多次 tick 才能完成的動作（前進 15 秒不可能在一次
// tick 內做完）。決策節點以 10 Hz 呼叫 tick()，每次都要立刻回傳
// RUNNING / SUCCESS / FAILURE，**絕對不要在 tick() 裡面 sleep**，那會把整個
// 決策節點卡住。
class StudentQualTask : public BT::ActionNodeBase {
public:
    StudentQualTask(const std::string& name, const BT::NodeConfiguration& config);

    // XML 上可以寫哪些屬性，由這個函式決定。
    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

    // 被上層中止時呼叫（例如 Timeout 到期、ReactiveFallback 切走）。
    // 一定要把速度歸零，否則載具會帶著最後一筆指令繼續衝。
    void halt() override;

    void setContext(std::shared_ptr<DecisionContext> ctx) { ctx_ = ctx; }

private:
    std::shared_ptr<DecisionContext> ctx_;

    // 跨 tick 的狀態。tick() 之間唯一活下來的東西就是成員變數。
    rclcpp::Time start_time_;
    bool started_ = false;
    double target_yaw_ = 0.0;
};
