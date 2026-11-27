# 故障实验室（FAQ）

> 20+ 个真实踩坑的速查表——全部来自本仓库的调试记录，按症状索引。
> 教学中建议先让学生复现故障，再找修复（每个坑在对应章节有展开）。

## 环境类

| 症状 | 根因 | 修复 |
|---|---|---|
| `gz: command not found` | Harmonic 与 Classic 的 /usr/bin/gz 冲突被 diversion | `ln -sf /usr/bin/gz-classic ~/.local/bin/gz` |
| Agent 报缺 libmicroxrcedds_agent.so | 装在 ~/.local 不在默认库路径 | `export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH` |
| colcon override 警告 | 编译前 source 了本工作空间 | 编译终端只 source humble |
| `scripts/run_simulation.sh: No such file` | 站在 ros2_ws/ 里 | `cd ..` 回仓库根目录 |
| 端口 8888 被占 | 旧栈孤儿进程 | 脚本 Ctrl-C 兜底清理；或按操作手册 pkill 组 |
| 关 Gazebo 窗口全栈停了 | on_exit_shutdown 设计 | 正常现象；脚本已兜底清理孤儿 |
| 磁盘被日志写满 | 无 TTY 时 pxh> 控制台死循环 | NO_PXH=1 + capped_log.sh 限幅 + 日志写大盘 |

## 控制类

| 症状 | 根因 | 修复 | 章节 |
|---|---|---|---|
| 节点 CPU 冻结、心跳停发 | RELIABLE 发布端在 FastDDS SHM 流控阻塞 | 发布端 BEST_EFFORT | 02 |
| 切 Offboard 被拒 | 未预流 setpoint | 先流 2s 再切模式 | 02 |
| 飞机贴地爬行 | EKF 原点重置，局部 z 误判"已起飞" | ekf_stable 门控(计数器稳定5s) | 04/07 |
| 键盘控制没反应 | 未按 b 开 Offboard / 步进 0.01 太小 | b→t→v；调参启动 | 01 |
| 降落判定永不完成 | x500 出生点偏移，地面 z≈-0.85 | 以上锁状态判定 | 02 |
| 多机 2 号机不解锁 | target_system ≠ MAV_SYS_ID | target_system=实例号+1 | 12 |
| 多机参数互相覆盖 | 共享工作目录 parameters.bson | 每实例独立工作目录(补丁0010) | 12 |

## 感知/SLAM 类

| 症状 | 根因 | 修复 | 章节 |
|---|---|---|---|
| echo 收得到、节点收不到 | QoS RELIABLE↔BEST_EFFORT 不匹配 | 订阅端 BEST_EFFORT | 03 |
| slam_toolbox 不建图不报错 | /clock 缺失/TF 断链/协方差缺失 | 四件套逐项查 | 07 |
| LIO 发散 | 点云含 inf | adapter 过滤 | 07 |
| LIO 拒收点云 | 缺 ring/time 字段 | lidar3d_adapter 补齐 | 07 |
| VIO 对齐失败 misalign | 合成图像+仿真IMU 不满足初始化假设 | 开放研究题 | 08 |
| YOLO ModuleNotFound | 系统/conda Python 混合 | run_yolo.sh 包装 | 05 |

## 视觉/规划类

| 症状 | 根因 | 修复 | 章节 |
|---|---|---|---|
| 精准降落越修越远 | 图像→世界映射符号错 | 逐级推导坐标系 | 06 |
| 绕靶画圈不收敛 | 误差未按航向旋转 | 偏航旋转 | 06 |
| 落点数字漂亮实际偏 | 用 PX4 局部系验证 | gz 世界系真值 | 06/16 |
| 避障把地面当障碍 | 深度图地面回波 | 只看图像上半部 | 09 |
| 规划器输出对但飞不对 | 局部系漂移失配 | 自标定映射 | 09 |
| 集群间距不足 FAIL | 同层交叉 | 高度分层 | 13 |
| 安全层疯狂误报 88° | 速度比近似倾角 | vehicle_attitude 四元数 | 11 |
| VTOL 过渡不执行 | 未先切 POSCTL | set_posctl_mode() 先行 | 14 |

## 判分类

| 症状 | 根因 | 修复 |
|---|---|---|
| 单次成功但复跑失败 | 幸存者偏差 | N 次通过率 ≥2/3 |
| pkill 杀了自己 | 模式匹配到自身命令行 | grade.py 用进程组管理 |

> 完整操作排障见[操作手册](OPERATIONS.md)；每个坑的原始日志在仓库 `logs/`。
