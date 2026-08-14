// 新生實作區（方案 B）：資格賽任務的自製行為樹節點。
//
// 這個檔案現在可以編譯、可以跑，但**什麼事都不會做** —— tick() 直接回報
// SUCCESS。你的工作是把下面四個 TODO 填完，讓載具真的下潛、前進、然後結束。
//
// 完整說明見 docs/FRESHMAN_QUAL_TASK.md。

#include "orca_decision/student_qual_task.hpp"

#include <cmath>

StudentQualTask::StudentQualTask(const std::string &name,
                                 const BT::NodeConfiguration &config)
    : BT::ActionNodeBase(name, config) {}

BT::PortsList StudentQualTask::providedPorts() {
  // TODO(1)：宣告這個節點在 XML 上收得到哪些參數。
  //
  // 目標是讓 trees.xml 可以這樣寫：
  //     <StudentQualTask duration="15.0" target_depth="0.5"/>
  //
  // 沒有宣告的屬性，getInput() 一定拿不到 —— BT.CPP 在載入 XML 時就會因為
  // 「多了不認識的屬性」而報錯。寫法照抄 blind_forward.cpp 的 providedPorts()。
  return {};
}

BT::NodeStatus StudentQualTask::tick() {
  // ctx_ 是整個決策堆疊的共用狀態：ROS node、世界模型、指令輸出口都掛在上面。
  // 正常情況下 main.cpp 註冊節點時就會用 setContext() 塞進來；這行是備援，
  // 從黑板上再拿一次（blind_forward.cpp 也是這樣寫的）。
  if (!ctx_) {
    config().blackboard->get("ctx", ctx_);
  }
  if (!ctx_) {
    return BT::NodeStatus::FAILURE;
  }

  // 這兩行會出現在 /orca/decision/status 上，是你在模擬跑起來之後唯一能
  // 「看見」行為樹走到哪裡的方式。實作時記得把 debug_msg 改成有用的內容。
  ctx_->current_action = name();
  ctx_->debug_msg = "StudentQualTask (未實作)";

  // TODO(2)：讀進 XML 傳來的參數。
  //     double duration;
  //     if (!getInput<double>("duration", duration)) { ... }
  //   讀不到要用 throw BT::RuntimeError 讓它大聲失敗，不要默默用預設值 ——
  //   靜默的預設值會讓「XML 改了卻沒生效」這種問題查上一整晚。

  // TODO(3)：第一次進來時記錄起點。
  //   started_ 為 false 時：
  //     - 用 ctx_->node->now() 記下 start_time_
  //     - 用 ctx_->world_model->getYaw() 記下 target_yaw_（要鎖航向的話）
  //     - 用 ctx_->desired_depth_pub 發出目標深度（參考 set_depth.cpp）
  //   注意 started_ 在動作結束與 halt() 時都必須歸位，否則節點被重跑一次
  //   （RetryUntilSuccessful、任務重新開始）時會沿用上一輪的起始時間，
  //   第二次進來會立刻結束。

  // TODO(4)：每次 tick 的主體。
  //   - 還沒到時間 → 用 ctx_->wrench_adapter->setCommand(cmd) 送出前進指令，
  //     回傳 BT::NodeStatus::RUNNING
  //   - 到時間了 → 把指令歸零、started_ = false、回傳 SUCCESS
  //
  //   MotionCommand 有 surge（前進）、sway（橫移）、heave（升降）、yaw（轉向）
  //   四個欄位，單位是控制堆疊那側的力／力矩，不是 m/s。數量級直接參考
  //   blind_forward.cpp 的 surge = 25.0f，自己亂填會不是原地不動就是暴衝。
  //
  //   深度不要用 heave 硬壓 —— 深度是控制堆疊的 PID 在管的，你只要用
  //   desired_depth_pub 告訴它「我要 0.5 公尺」，剩下的它會處理。

  RCLCPP_WARN_ONCE(
      ctx_->node->get_logger(),
      "StudentQualTask 尚未實作：節點直接回報 SUCCESS，載具不會有任何動作。"
      "請完成 src/behavior_tree_nodes/student_qual_task.cpp 裡的 TODO(1)~(4)。");

  return BT::NodeStatus::SUCCESS;
}

void StudentQualTask::halt() {
  started_ = false;
  if (ctx_) {
    // 空的 MotionCommand 就是「四個方向都不出力」。
    ctx_->wrench_adapter->setCommand(MotionCommand());
  }
}
