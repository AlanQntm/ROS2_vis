# ROS2 视觉最小可运行实现（Armor MVP + ROS 包）

本仓库完成了一个基于 OpenCV 的传统视觉最小可运行版本（armor_mvp）：

- 支持从本地 USB 相机或视频文件取流；
- 在每帧中输出候选装甲板的外接框、四角点、score，并通过 IOU+EMA 做稳定跟踪（track_id 连续）；
- 提供调试可视化（叠加 bbox、角点顺序、id、score、FPS），可保存为视频；
- 提供离线测试脚本化用法，支持导出逐帧 JSONL 结果。

## 目录

- 方案文档：`docs/docs.md`
- 最小实现：`armor_mvp/`
  - 代码入口：`armor_mvp/main.cpp`
  - 配置：`armor_mvp/config/app.yaml`
  - 可执行：`armor_mvp/build/armor_mvp`
- ROS2 封装：`armor_vision/`
  - 自定义消息：`msg/ArmorDetection.msg`, `msg/ArmorDetections.msg`
  - 节点：`include/armor_vision/armor_vision_node.hpp`, `src/armor_vision_node.cpp`
  - 检测/跟踪：`src/detector.cpp`, `src/tracker.cpp`
  - 启动文件：`launch/bringup_detect.launch.py`
  - 配置：`config/camera_info.yaml`

## 构建与运行（本地 OpenCV 可执行）

```bash
cmake -S armor_mvp -B armor_mvp/build -DCMAKE_BUILD_TYPE=Release
cmake --build armor_mvp/build -j

# 方式一：直接指定视频，关闭窗口、保存输出、导出 JSONL
cd armor_mvp
./build/armor_mvp --video ../blue.mp4 --save out.mp4 --no-window --dump results.jsonl

# 方式二：使用配置文件（路径示例已指向 ../blue.mp4）
./build/armor_mvp --config config/app.yaml --no-window
```

## 离线测试结果（本地可执行）

- 输入示例视频（仓库自带）：[blue.mp4](blue.mp4)
- 离线运行结果视频（叠加可视化）：[armor_mvp/out.mp4](armor_mvp/out.mp4)
- 逐帧结构化输出（JSONL）：[armor_mvp/results.jsonl](armor_mvp/results.jsonl)

> 说明：`blue.mp4` 并非装甲板素材，检测结果为空属预期；若提供包含装甲板的样例视频，可直接复现框与角点输出。

## 参数说明

- 参数项与默认值与 要求 中“参数配置”保持一致（`bin.* / morph.ksize / filter.* / detector.max_candidates / tracker.* / ui.*`）。
- 数据结构：每个轨迹包含 `id/score/bbox(TLWH)/corners[TL,TR,BR,BL]`；帧级包含 `fps`。
- 可扩展：如需 ROS2 版本（`armor_vision` 包），可在此基础上封装为节点并发布两个话题（detections/debug_image）。

## ROS2（Humble）构建与运行

```bash
# 在你的 ROS2 工作空间（例如 ~/ros2_ws）下，把本包放入 src/ 目录
# 假设当前仓库路径为 ~/ros2_vis
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
ln -s ~/ros2_vis/armor_vision .

cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 相机
ros2 launch armor_vision bringup_detect.launch.py device_id:=0

# 视频（示例：使用本仓库 blue.mp4）
ros2 launch armor_vision bringup_detect.launch.py video_path:=${HOME}/ros2_vis/blue.mp4

# 查看话题
rqt_image_view /armor/debug_image
ros2 topic echo /armor/detections --once
```

## 参考

- 文档：`docs/docs.md`
- 代码入口：`armor_mvp/main.cpp`
- 配置模版：`armor_mvp/config/app.yaml`
