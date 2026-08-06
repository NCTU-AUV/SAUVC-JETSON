# SAUVC-JETSON —— Orca AUV 感知與決策堆疊

跑在 **Jetson Orin NX + Isaac ROS 3.2** 的 ROS 2 workspace：
相機輸入 → 物件偵測 → 深度估計 → BehaviorTree 任務執行 → wrench 指令。

**平常不用直接開這個 repo。** 從 [super-repo](https://github.com/NCTU-AUV/SAUVC)
一個指令就會把這個堆疊連同控制與模擬一起拉起來：

```shell
cd ../          # SAUVC super-repo
make up && make build
make sim ARENA=finals SEED=2
```

底下是單獨開發本 repo 時用的流程。系統怎麼運作見 [ARCHITECTURE.md](ARCHITECTURE.md)。

---

## Packages

| Package | 位置 | 職責 |
|---|---|---|
| [`orca_interface`](orca_interface/) | `orca_interface/` | 共用訊息（`PerceptionObject`、`PerceptionArray`、`DecisionStatus`） |
| [`camera_selector`](perception_pipeline/camera_selector/) | `perception_pipeline/` | 前相機 ↔ 底部相機即時切換（composable） |
| [`isaac_ros_yolov8`](perception_pipeline/isaac_ros_yolov8/) | `perception_pipeline/` | YOLOv8 TensorRT 解碼 + 視覺化 |
| [`depth_perception`](perception_pipeline/depth_perception/) | `perception_pipeline/` | 深度估計 + 多幀穩定度追蹤 |
| [`orca_perception`](perception_pipeline/orca_perception/) | `perception_pipeline/` | 感知管線的統一 launcher 與 YAML |
| [`orca_decision`](orca_decision/) | `orca_decision/` | BehaviorTree.CPP 任務決策 |

---

## 單獨開發

### 1. 前置

依 [Isaac ROS Getting Started](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/index.html)
完成 **Compute Setup** 與 **Developer Environment Setup**（注意平台是 x86 還是 Jetson）。

### 2. Clone 並攤平進 `src/`

```bash
cd $ISAAC_ROS_WS/src
git clone --recurse-submodules git@github.com:NCTU-AUV/SAUVC-JETSON.git
mv SAUVC-JETSON/* . && mv SAUVC-JETSON/.g* . && rm -rf SAUVC-JETSON
```

### 3. 設定映像層並進容器

```bash
touch ~/.isaac_ros_common-config
echo "CONFIG_IMAGE_KEY=ros2_humble.realsense.orca25" >> ~/.isaac_ros_common-config
echo "alias isa='cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && ./scripts/run_dev.sh'" >> ~/.bashrc
source ~/.bashrc
isa    # 第一次會 build 映像（約 40 分鐘），之後直接進 bash
```

從任何新終端機再跑 `isa` 就會接進**同一個容器**。

> **能不在 Jetson 上 build 就不要。** 映像約 20 GB，其中 86% 來自 NVIDIA 的
> base layer（Triton + CUDA devel + PyTorch + TensorRT），沒有可以砍的空間。
> 在 x86 機器建一次推 registry，其他地方只 pull：
>
> ```bash
> ./scripts/orca_registry.sh build --arm64   # x86 建置機
> ./scripts/orca_registry.sh push
> ./scripts/orca_registry.sh pull            # Jetson
> ```
>
> **也絕對不要改 `docker/Dockerfile.base`、`.aarch64` 或 `.ros2_humble`。**
> `build_image_layers.sh` 用 Dockerfile 鏈的 md5 去查 NVIDIA 的預建映像；
> 改動任何一個都會讓查詢失效，逼 Jetson 從頭 build 17.8 GB 的 base，要好幾個小時。
> 專案自己的改動放最外層的 `docker/Dockerfile.orca25`，不影響那個 hash。

### 4. Build

```bash
cd /workspaces/isaac_ros-dev
colcon build --symlink-install
source install/setup.bash
```

---

## 啟動

**一個指令就好** —— `autonomy.launch.py` 同時拉起感知管線與決策節點：

```bash
cd src/orca_decision                                  # trees.xml 是相對路徑，必須從這裡啟動
ros2 launch orca_decision autonomy.launch.py
```

模擬情境要指定感知設定（相機話題掛在載具 namespace 底下，與實機不同）：

```bash
ros2 launch orca_decision autonomy.launch.py perception_config:=simulation_params.yaml
```

| 參數 | 預設 | 說明 |
|---|---|---|
| `perception_config` | 空（實機） | 裸檔名，解析到 `orca_perception/config/`。模擬用 `simulation_params.yaml` |
| `use_perception` | `true` | `false` 只跑行為樹 |
| `use_viz` | 用 YAML 值 | 感知視覺化，需要 X server |
| `namespace` | `$ORCA_NAMESPACE` | 載具 namespace，決定 remap 目標與 YAML 裡 `$(ns)` 的展開 |

> **不要分別跑 `perception.launch.py` 與 `decision.launch.py`。**
> `autonomy.launch.py` 已經包含前者，兩個一起跑會有兩條感知管線同時搶 GPU 與話題。
> `decision.launch.py` 單獨跑則完全沒有感知 —— 世界模型永遠是空的，所有搜尋／逼近
> 節點跑到逾時，而 `ros2 node list` 全都顯示健康。

啟動任務：

```bash
ros2 topic pub --once /orca/decision/start_mission std_msgs/msg/Bool 'data: true'
```

載具要能動，控制堆疊那側還得先 arm，見 [super-repo README](../README.md#使用)。

---

## 除錯

```bash
ros2 topic echo /orca/decision/status              # 任務階段、目前 BT 動作、目標鎖定
ros2 topic echo /orca/perception_array             # 感知輸出
ros2 topic hz /orca/selected/image_raw             # 影像有沒有進來
ros2 param get /decision_node main_tree_id         # 現在跑哪棵樹
```

首次啟動 TensorRT 會重建 CUDA engine，**要好幾分鐘**，期間
`/orca/perception_array` 不會有資料。這不是故障。

---

## 設定

| 檔案 | 內容 |
|---|---|
| `perception_pipeline/orca_perception/config/perception_params.yaml` | 感知管線（實機話題） |
| `perception_pipeline/orca_perception/config/simulation_params.yaml` | 感知管線（模擬話題，用 `$(ns)`） |
| `orca_decision/config/decision_params.yaml` | 決策參數 |
| `orca_decision/config/trees.xml` | 任務樹 |

`model_profile`（感知）與 `main_tree_id`（決策）是兩個必須手動同步的真相來源。

---

## 模擬

模擬有自己的 repo 與容器：[SAUVC-Simulation](https://github.com/NCTU-AUV/SAUVC-Simulation)。
在那邊起 Gazebo，這邊用 `simulation_params.yaml` 啟動 —— 兩個容器共用同一個 ROS 2 graph。

Gazebo 已刻意**不再**裝進本映像（以前有，與模擬容器重複），舊的 `sauvc_sim`
package 移到 [`legacy/`](legacy/) 且不編譯。

> 跨容器 discovery 需要 `RMW_IMPLEMENTATION`、`ROS_DOMAIN_ID` 與 DDS transport
> 三者在每個容器都一致，特別是 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`。
> 少了它 Fast DDS 會宣告其他容器搆不到的共享記憶體 locator，participant 互相
> match 到但資料永遠走不通，**而且沒有任何錯誤訊息**。

---

## 相關文件

- [ARCHITECTURE.md](ARCHITECTURE.md) —— 系統怎麼運作
- [../docs/HANDOFF.md](../docs/HANDOFF.md) —— 座標慣例、已知缺陷、驗收方式
- [../README.md](../README.md) —— super-repo：一次啟動整套系統
