# legacy/

這個目錄裡的 package **不會被編譯**（同層的空檔案 `COLCON_IGNORE` 會讓 colcon
跳過整個子樹）。原始碼保留在 git 裡供查閱與日後取回。

放在這裡的東西有兩個共同點：已經被更好的實作取代，而且**留在編譯路徑上會拖慢
每一次 `colcon build`** —— 在 Jetson 上這件事很有感。

| Package | 被誰取代 | 說明 |
|---|---|---|
| `sauvc_sim` | 獨立的 `SAUVC-Simulation` repo | Gazebo Fortress 場景、模型與 teleop 腳本。原本就已標註 deprecated |
| `fsm_decision` | `orca_decision`（BehaviorTree.CPP） | 更早期的手刻有限狀態機決策實作 |

## 一併移除的東西

- **`realsense-ros` submodule。** 與 `isaac_ros_common/docker/Dockerfile.realsense`
  重複 —— 該層已經把 NVIDIA 的 realsense-ros fork（`NVIDIA-ISAAC-ROS/realsense-ros`）
  bloom 打包後裝進映像。workspace 裡再放一份 Intel 上游版本只會造成混淆，
  而且該 submodule 從未初始化過（目錄是空的）。
  要改 RealSense 驅動請改 `Dockerfile.realsense` 的 `REALSENSE_ROS_GIT_URL`
  與 `REALSENSE_ROS_VERSION`。

- **`isaac_ros_common/docker/Dockerfile.orca`。** 已被 `Dockerfile.orca25` 取代，
  沒有任何 image key 會解析到它。內容還多裝了本專案不用的
  `isaac_ros_detectnet` 與 `isaac_ros_triton`。

## 要取回的話

刪掉 `legacy/COLCON_IGNORE`，把需要的 package 移回上層目錄。

`sauvc_sim` 若要取回，還要恢復 `isaac_ros_common/docker/Dockerfile.orca25` 裡的
Gazebo 安裝層與 `scripts/entrypoint_orca25.sh` 的 `GZ_SIM_RESOURCE_PATH`。
但正常情況下不需要 —— 模擬請用 `SAUVC-Simulation` 那個獨立容器，
Isaac 容器裡不該再裝一套 Gazebo。
