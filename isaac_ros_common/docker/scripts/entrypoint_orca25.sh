# 進入容器時自動 source workspace。
#
# 這裡曾經還會設定 GZ_SIM_RESOURCE_PATH 指向 src/sauvc_sim 的 models/ 與
# meshes/。sauvc_sim 已移入 legacy/（模擬改用獨立的 SAUVC-Simulation 容器），
# Gazebo 也不再裝進本映像，所以那行一併移除。
source install/setup.bash
