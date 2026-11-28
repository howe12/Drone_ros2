# 图片来源与许可（CREDITS）

本目录图片分三类，**使用前请确认来源类型**。

## 1. 官方文档图（`external/`）

来自公开的权威技术文档/仓库，已下载到本地（不热链），按原许可使用并署名。

| 文件 | 出处 | 许可 | 用在 |
|---|---|---|---|
| `external/px4-uxrce-dds-ros2-arch.svg` | PX4 User Guide · [uXRCE-DDS (PX4-ROS 2 Bridge)](https://docs.px4.io/main/en/middleware/uxrce_dds.html) | CC BY 4.0 | 第 1 章 图 1.3 |
| `external/px4-mc-control-arch.jpg` | PX4 User Guide · [Controller Diagrams](https://docs.px4.io/main/en/flight_stack/controller_diagrams.html) | CC BY 4.0 | 第 00 章 图 0.3 |
| `external/px4-enu-ned.png` | PX4-Autopilot 仓库 · `docs/assets/advanced/ENU-NED.png` | CC BY 4.0 | 第 2 章 图 2.3 |
| `external/px4-frame-quadrotorx.svg` | PX4-Autopilot 仓库 · `docs/assets/airframes/types/QuadRotorX.svg` | CC BY 4.0 | 第 00 章 图 0.1 |
| `external/px4-frame-plane.svg` | PX4-Autopilot 仓库 · `docs/assets/airframes/types/Plane.svg` | CC BY 4.0 | 第 00 章 图 0.1 |
| `external/px4-frame-vtolplane.svg` | PX4-Autopilot 仓库 · `docs/assets/airframes/types/VTOLPlane.svg` | CC BY 4.0 | 第 00 章 图 0.1 |

**署名要求**：CC BY 4.0 允许修改与商用，但必须注明来源与许可。正文图注下方已在斜体行标注出处；新增官方图时请照此办理，并更新本表。

## 2. 本仓库实测截图（`images/*.png`）

我们自己跑仿真截的图（Gazebo 画面、RViz、终端输出、判分报告），版权归本仓库，随仓库许可发布。

占位文件以 `PLACEHOLDER-` 开头，表示**待补**：

- `PLACEHOLDER-fig-*`：待生图的概念示意图（提示词写在正文 `IMAGE_PLACEHOLDER` 块里）
- `PLACEHOLDER-shot-*`：待实际运行截图（命令写在正文 `SCREENSHOT_PLACEHOLDER` 块里）

## 3. 待生成的概念图（占位）

目前仅剩：

| 图 | 为什么没有官方版 |
|---|---|
| 图 0.2 姿态三轴（Roll/Pitch/Yaw） | 已查 PX4 User Guide 全部图、PX4-Autopilot `docs/assets/` 全树、ArduPilot；无多旋翼三轴示意图（仅固定翼 pitch 单轴）。Wikimedia Commons 当前网络不可达 |
| 图 1.4 首飞四阶段 | 绑定本课程日志与状态位，官方不可能有 |
| 图 2.2 Offboard 闭环 | 绑定本课程话题名（`/x500_0/fmu/in|out/...`） |
| 图 2.4 offboard_demo 状态机 | 本课程代码的状态机，官方不可能有 |

## 使用规则

1. **优先官方图**：PX4 / Gazebo / ROS 2 / ArUco 官方文档或仓库里有的，不要再生成。
2. **必须署名**：官方图在图注下方给出处与许可，并登记到本表。
3. **不热链**：一律下载到 `external/`，避免上游改版或断链导致课程挂图。
4. **不要用来源不明的图**：博客、公众号、二手教程的图，未经确认许可不得使用。
5. 官方图标（SVG）尺寸不一，正文用 `<img width="…">` 控制，别直接铺满。
