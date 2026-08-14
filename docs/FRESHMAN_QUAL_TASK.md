# 新生實作：資格賽任務 —— 提示說明書

> 這份文件在 **`edu/newbie-2026` 教材分支**上，不會合併回 `main`。
> 你在這裡改壞任何東西都不會影響比賽用的程式碼，放心動手。

任務是資格賽的標準流程：**下潛到指定深度 → 直行通過閘門 → 上浮**。
兩種難度任選，建議兩個都做過一遍：

| | 方案 A | 方案 B |
|---|---|---|
| 改什麼 | 只改 `orca_decision/config/trees.xml` | 寫一個 C++ 行為樹節點 |
| 需要重新編譯 | 不用 | 每次都要 |
| 學到什麼 | 行為樹怎麼組、現有節點有哪些 | ROS 2 節點怎麼下指令、BT 節點的生命週期 |
| 預估時間 | 30 分鐘 | 3～6 小時 |

---

## 0. 環境

```bash
git clone --recurse-submodules -b edu/newbie-2026 https://github.com/NCTU-AUV/SAUVC.git
cd SAUVC
git -C SAUVC-JETSON checkout edu/newbie-2026    # 見下方說明，不要跳過
make up && make build
```

**第三行不能省。** submodule 被 clone 下來時停在 detached HEAD（不在任何分支上），
這種狀態下你 commit 的東西不屬於任何分支，之後只要一個 `git checkout`
就會整批消失而且沒有任何警告。你要改的檔案全都在這個 submodule 裡面。

`make build` 也不能省。install space 是 docker named volume，映像裡沒有預先建置過，
沒 build 過的檔案根本不在容器裡。

以下所有指令都在 super-repo 根目錄（`SAUVC/`）執行。決策程式碼在
`SAUVC-JETSON/orca_decision/`。

---

## 1. 先搞懂資料往哪裡流

不理解這張圖就開始寫，會卡在「程式明明跑了但船不動」。

```text
  Gazebo ──/orca_auv/sensors/imu──►  WorldModel ──getYaw()──┐
                                                            │
                                              你的 BT 節點 ─┤
                                                            │
        ctx_->wrench_adapter->setCommand(cmd) ──────────────┘
                    │
                    ▼  50 Hz
        /orca_auv/control/wrench_sources/decision ──► 控制堆疊 ──► 推進器
                    ▲
        ctx_->desired_depth_pub
                    │
        /orca_auv/control/targets/depth_m ──► 深度 PID ──► 推進器
```

三個必須記住的點：

1. **行為樹以 10 Hz 被 tick 一次**（`decision_node.cpp` 的 `bt_timer_`）。
   每次 `tick()` 都必須**立刻**回傳 `RUNNING` / `SUCCESS` / `FAILURE`。
   在 `tick()` 裡 `sleep` 會把整個決策節點凍住 —— 這是最常見的第一個錯誤。

2. **你不直接控制推進器。** 你把「我想往前」寫進 `MotionCommand`，
   丟給 `wrench_adapter`，它以 50 Hz 把濾波後的結果發出去。
   你的節點沒在跑的時候，發出去的是零。

3. **深度不歸你管。** 深度是控制堆疊的 PID 在維持的，
   你只要用 `desired_depth_pub` 說「我要 0.5 公尺」就好。
   想用 `heave` 硬壓深度會跟 PID 打架。

---

## 2. 方案 A：只改 XML

### 要改的檔案

只有一個：[`orca_decision/config/trees.xml`](../orca_decision/config/trees.xml)，
找到 `<BehaviorTree ID="StudentSimpleQualMission">`，把裡面兩個 `TODO(A-*)` 填掉。

### 可以用的積木

| 標籤 | 屬性 | 行為 |
|---|---|---|
| `<SetDepth/>` | `depth`（公尺） | 發一次目標深度就結束，**立刻**回 SUCCESS，不會等下潛完成 |
| `<BlindForward/>` | `duration`（秒）、`heading_lock`（true/false） | 固定推力前進，期間持續回 RUNNING，時間到回 SUCCESS |
| `<TurnToYaw/>` | `degrees` | 相對目前航向轉幾度 |

`<Sequence>` 由左到右執行，前一個 SUCCESS 才輪到下一個，任何一個 FAILURE 整串中止。

> **`SetDepth` 立刻回 SUCCESS 這件事很重要。** 它不是「下潛到 0.5 公尺才繼續」，
> 而是「講完就走」。所以下一個動作開始時，船其實還在往下沉。這是現有節點的
> 設計，不是 bug —— 想想看你要不要在中間補點什麼。

### 跑起來

```bash
make sim ARENA=qualification SEED=2 PERCEPTION=false TREE=StudentSimpleQualMission
```

- `PERCEPTION=false` 只跑行為樹，不啟動 YOLO。方案 A 用不到視覺，
  而且第一次啟動感知要等 TensorRT 建 CUDA engine，好幾分鐘。
- `TREE=` 指定跑哪棵樹，不用改 YAML 也不用重新 build。
- `SEED` 固定場地佈局。除錯時務必給，否則每次重啟道具位置都不一樣。

改完 XML **不需要重新 build**（install space 是逐檔符號連結，指回原始檔），
但要重啟決策節點才會重新載入：

```bash
make launch_autonomy SIM=true PERCEPTION=false TREE=StudentSimpleQualMission
```

### 讓船真的動起來

模擬起來之後船還是不會動 —— 開機預設是 `SAFE_DISABLED`，推進器沒有輸出。
先解鎖，再發任務開始：

```bash
docker compose exec control bash -lc 'source /opt/ros/humble/setup.bash && source /root/rpi_ros2_ws/install/setup.bash && ros2 service call /orca_auv/system_manager/set_mode/depth_hold std_srvs/srv/Trigger && ros2 service call /orca_auv/system_manager/set_mode/autonomous std_srvs/srv/Trigger'
```

```bash
docker compose exec autonomy bash -lc 'source /opt/ros/humble/setup.bash && source /workspaces/isaac_ros-dev/install/setup.bash && ros2 topic pub --once /orca/decision/start_mission std_msgs/msg/Bool "{data: true}"'
```

### 驗收

在 Gazebo 畫面上看得到三段動作：下沉 → 直線前進一段時間 → 浮回水面。
同時開另一個終端機看行為樹走到哪：

```bash
docker compose exec autonomy bash -lc 'source /opt/ros/humble/setup.bash && source /workspaces/isaac_ros-dev/install/setup.bash && ros2 topic echo /orca/decision/status'
```

`current_action` 應該依序出現 `SetDepth` → `BlindForward` → `SetDepth`，
最後 autonomy 的 log 印出 `Mission completed successfully.`。

> 能不能真的穿過閘門，取決於起始線位置和你給的秒數 —— 那是下一步的題目，
> 不是方案 A 的驗收標準。方案 A 只要求三段動作照順序發生。

---

## 3. 方案 B：寫自己的 C++ 節點

### 檔案地圖

| 檔案 | 你要做的事 |
|---|---|
| [`include/orca_decision/student_qual_task.hpp`](../orca_decision/include/orca_decision/student_qual_task.hpp) | 已經寫好，先讀懂。要加成員變數就加在這 |
| [`src/behavior_tree_nodes/student_qual_task.cpp`](../orca_decision/src/behavior_tree_nodes/student_qual_task.cpp) | **主戰場**，TODO(1)～(4) |
| [`src/main.cpp`](../orca_decision/src/main.cpp) | TODO(5)，把節點註冊給行為樹引擎 |
| [`config/trees.xml`](../orca_decision/config/trees.xml) | 取消 `StudentMission` 那段的註解 |

參考對象是 [`blind_forward.cpp`](../orca_decision/src/behavior_tree_nodes/blind_forward.cpp)。
它做的事和你的作業高度重疊，卡住就去讀它 —— 但不要整個複製貼上，
你的節點要同時處理深度和前進，它只處理前進。

### 順序很重要

**先做 TODO(5) 註冊，最後才取消 XML 的註解。** 反過來的話，BT.CPP 在載入
XML 的當下就會逐一檢查節點名稱有沒有註冊過，遇到不認識的 `<StudentQualTask>`
就讓**整份 trees.xml 載入失敗**，連比賽用的樹一起陪葬：

```text
[ERROR] [decision_node]: Failed to load BehaviorTree: Error at line 202: -> Node not recognized: StudentQualTask
```

看到這行就是順序反了，或是註冊的字串跟 XML 標籤沒有一字不差。

### API 速查

| 你想做的事 | 寫法 |
|---|---|
| 讀 XML 傳來的參數 | `getInput<double>("duration", duration)` |
| 現在幾點 | `ctx_->node->now()`（回傳 `rclcpp::Time`，相減得 `.seconds()`） |
| 目前航向 | `ctx_->world_model->getYaw()`（弧度） |
| 送出運動指令 | `ctx_->wrench_adapter->setCommand(cmd)` |
| 設定目標深度 | `ctx_->desired_depth_pub->publish(msg)`（`std_msgs::msg::Float64`） |
| 印訊息 | `RCLCPP_INFO(ctx_->node->get_logger(), "...")` |
| 讓狀態出現在 `/orca/decision/status` | 寫 `ctx_->current_action` 和 `ctx_->debug_msg` |

`MotionCommand` 有 `surge`（前進）、`sway`（橫移）、`heave`（升降）、`yaw`（轉向）
四個 float。**單位不是 m/s**，是丟給控制堆疊的力／力矩。數量級直接參考
`blind_forward.cpp` 的 `surge = 25.0f`；自己憑感覺填不是原地不動就是暴衝。

### 四個容易踩的坑

1. **`providedPorts()` 沒宣告的屬性，XML 上寫了會直接載入失敗。**
   宣告和 XML 要對得起來。

2. **`started_` 一定要在結束時歸位。** 動作做完回 SUCCESS 前、以及 `halt()`
   裡都要設回 `false`。漏掉的話節點被重跑第二次（`RetryUntilSuccessful`、
   任務重新開始）會沿用上一輪的 `start_time_`，一進去就判定「時間到了」而
   立刻結束。

3. **`halt()` 要把速度歸零。** 被上層中止時如果沒歸零，最後一筆指令會留在
   `wrench_adapter` 裡，船會帶著它繼續衝。

4. **參數讀不到要大聲失敗**（`throw BT::RuntimeError`），不要默默套預設值。
   靜默的預設值會讓「XML 改了卻沒生效」查上一整晚。

### 跑起來

C++ 改完**一定要重新編譯**：

```bash
make build
make launch_autonomy SIM=true PERCEPTION=false TREE=StudentMission
```

（模擬本身還在跑的話不用重開 Gazebo，重啟 autonomy 就好。）

### 驗收

和方案 A 同樣的三段動作，但 `/orca/decision/status` 的 `current_action`
應該顯示的是 `StudentQualTask` 而不是內建節點。
沒填 TODO 之前節點會在 log 印一行警告然後直接回 SUCCESS，船不會動 ——
看到那行就代表你跑的是骨架，不是你的實作。

---

## 4. 卡關對照表

| 症狀 | 原因 | 怎麼辦 |
|---|---|---|
| `Failed to load BehaviorTree: Node not recognized` | XML 用了沒註冊的節點 | 先做 main.cpp 的 TODO(5)，檢查字串一字不差 |
| `Failed to load BehaviorTree: ... must have at least 1 child` | `<Sequence>` 被清空了 | 至少留一個子節點 |
| 一切正常但船完全不動 | 沒 arm | 跑上面那條 `set_mode` 服務呼叫 |
| 船不動，log 有「尚未實作」警告 | 跑的是骨架 | TODO(1)～(4) 還沒填 |
| 改了 `.cpp` 卻沒生效 | 沒重新編譯 | `make build` |
| 改了 XML 卻沒生效 | 節點沒重啟 | `make launch_autonomy ...` |
| 換了 `TREE=` 卻跑到別棵樹 | 確認實際生效的值 | `ros2 param get /decision_node main_tree_id` |
| 船一直往一邊偏 | 沒鎖航向 | `heading_lock="true"`，或自己實作航向 P 控制 |
| Gazebo 開不起來 | X server 授權 | `make sim HEADLESS=true` |

看 log：

```bash
make logs_autonomy      # 決策節點
make logs_control       # 控制堆疊
make logs_sim           # Gazebo
make status             # 容器 / 節點 / 模式一覽
```

---

## 5. 做完之後

1. **加入感測器回饋**：讓節點訂閱深度或視覺資訊，看到閘門才前進、
   到達深度才進下一步，而不是盲走固定秒數。
   現成的參考是 `search_target.cpp` 和 `approach_target.cpp`。
2. **理解 Reactive 節點**：讀 `QualificationMission` 裡的 `ReactiveFallback`，
   想清楚為什麼避障要放在那個位置才會每個 tick 都被檢查。
3. 把你的節點併進 `behavior_tree_nodes.hpp`，走正式的 PR 流程。

---

## 附錄：註冊節點的樣板

這段是 repo 的固定寫法，不是這次作業的重點。真的卡住再看。

`main.cpp` 最上面加 include：

```cpp
#include "orca_decision/student_qual_task.hpp"
```

在 `RegisterBehaviorTreeNodes()` 的 TODO(5) 位置加：

```cpp
factory.registerBuilder<StudentQualTask>("StudentQualTask",
    [ctx](const std::string& name, const BT::NodeConfiguration& config) {
        auto node = std::make_unique<StudentQualTask>(name, config);
        node->setContext(ctx);
        return node;
    });
```

`setContext(ctx)` 那行是關鍵。漏掉一樣編得過、一樣載入得了，
但節點裡的 `ctx_` 會是空的 —— 你會拿不到世界模型也送不出指令。
