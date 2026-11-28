# 术语表（GLOSSARY）

> 全课程概念的白话速查。每条 = 一句话解释 + 深入章节。
> 章节正文遇到术语时，要么就地解释，要么链接到这里。新增术语必须同步登记。

## 飞行器与飞行

| 术语 | 白话解释 | 深入 |
|---|---|---|
| 多旋翼 | 靠多个螺旋桨直接向上拉产生升力的飞行器，能悬停 | [00](course/00-basics.md) |
| 固定翼 | 靠机翼在前进气流中产生升力，不能悬停，续航长 | [00](course/00-basics.md) |
| VTOL / 复合翼 | 垂直起降（多旋翼模式）+ 固定翼巡航的混合飞行器 | [00](course/00-basics.md)、[14](course/14-vtol.md) |
| 四旋翼 / 六旋翼 | 4 / 6 个旋翼的多旋翼；桨越多冗余越高 | [00](course/00-basics.md) |
| 姿态（Attitude） | 飞机在空中的朝向，用 Roll/Pitch/Yaw 三个角描述 | [00](course/00-basics.md) |
| Roll / Pitch / Yaw | 横滚（左右侧倾）/ 俯仰（机头抬低）/ 偏航（原地转向） | [00](course/00-basics.md) |
| 差速 | 四旋翼靠四个桨的转速差实现所有动作 | [00](course/00-basics.md) |
| 轴距 | 对角电机间距；x500 的 500 即 500mm 轴距 | [00](course/00-basics.md) |
| 机架（airframe） | PX4 里"机型+参数"的配置模板，决定控制器结构；课程自建 4012/4013/4014 | [03](course/03-sensors.md) |

## 飞控与估计
| uORB | PX4 内部的发布订阅总线，飞控各模块靠它交换数据 | [00](course/00-basics.md) |
| HITL | Hardware-In-The-Loop：飞控用真实硬件、传感器用仿真 | [00](course/00-basics.md) |
| PWM / DShot | 两种电调信号：PWM 用占空比，DShot 是数字协议（更精确抗干扰） | [00](course/00-basics.md) |
| QGC | QGroundControl，地面站软件，用于参数配置与校准 | [00](course/00-basics.md) |

| 术语 | 白话解释 | 深入 |
|---|---|---|
| 飞控 | 飞机上的控制计算机：稳姿态、去目标、估状态、管模式 | [00](course/00-basics.md) |
| 姿态控制（内环） | 保持目标 Roll/Pitch/Yaw，输出电机转速，数百 Hz–1kHz | [00](course/00-basics.md) |
| 位置控制（外环） | 把位置目标翻译成目标姿态，数十 Hz | [00](course/00-basics.md) |
| EKF | 扩展卡尔曼滤波：融合 GPS/IMU/气压计等矛盾测量，估计位置/速度/姿态 | [00](course/00-basics.md)、[02](course/02-offboard.md) |
| EKF 原点重置 | GPS 跳变时 EKF 重新锚定局部原点，局部坐标"瞬移"；判分要用世界系真值 | [02](course/02-offboard.md)、[04](course/04-waypoints.md) |
| IMU | 惯性测量单元：加速度计+陀螺仪，测加速度与角速度 | [03](course/03-sensors.md) |
| 模式管理 | 飞控状态机：同一时刻一种模式（Manual/Position/Offboard/RTL/LAND…） | [00](course/00-basics.md) |
| Offboard | "板外"模式：控制权交给外部程序，需持续心跳+设定点 | [02](course/02-offboard.md) |
| 心跳（Heartbeat） | 持续发布的 OffboardControlMode，证明外部控制器活着（>2Hz） | [02](course/02-offboard.md) |
| 设定点（Setpoint） | 希望飞机达到的目标位置/速度（TrajectorySetpoint） | [02](course/02-offboard.md) |
| 预流（Pre-stream） | 切 Offboard 前先持续发设定点（课程 2s），否则被拒 | [02](course/02-offboard.md) |
| 失效保护（failsafe） | 失控/低电量/外部掉线时自动保命（返航/降落） | [00](course/00-basics.md)、[02](course/02-offboard.md) |
| 解锁（Arm） | 允许电机转动；上锁（Disarm）则电机停 | [01](course/01-setup.md) |
| 预检（Preflight） | 解锁前的健康检查（EKF/GPS/传感器），不过则拒绝解锁 | [01](course/01-setup.md) |
| SITL | Software-In-The-Loop：固件编译成电脑程序+仿真传感器，与真机同源码 | [00](course/00-basics.md)、[01](course/01-setup.md) |
| MAVLink | 无人机通信协议；PX4 命令（解锁/切模式/返航）走它 | [02](course/02-offboard.md) |
| MAV_SYS_ID | PX4 实例编号；多机命令的 target_system 必须等于它（实例号+1） | [12](course/12-formation.md) |
| 地理围栏（Geofence） | 虚拟边界，越界自动触发返航/降落 | [11](course/11-safety.md) |

## 坐标与数学

| 术语 | 白话解释 | 深入 |
|---|---|---|
| NED | 北-东-地，航空惯例，z 向下为正；5m 高度 = z=-5 | [00](course/00-basics.md)、[02](course/02-offboard.md) |
| ENU | 东-北-上，ROS/Gazebo 惯例，z 向上为正 | [02](course/02-offboard.md) |
| 机体系（FLU） | 以飞机自身为参照：前-左-上 | [09](course/09-planning.md) |
| 光心系 | 相机坐标：x 右、y 下、z 沿光轴向前 | [06](course/06-precision-landing.md) |
| 世界系真值 | Gazebo 给出的绝对位置，不随 EKF 重置变化；判分用它 | [16](course/16-grading.md) |
| 四元数 | 用 4 个数表示旋转，无万向节死锁；姿态话题用它 | [11](course/11-safety.md) |
| 协方差 | 估计的不确定度；SLAM 需要带协方差的里程计 | [07](course/07-slam.md) |

## 通信与软件
| slam_toolbox | ROS2 的 2D 激光 SLAM 实现（图优化类），输出栅格地图 | [07](course/07-slam.md) |

| 术语 | 白话解释 | 深入 |
|---|---|---|
| ROS2 节点 | 一个任务程序进程，通过话题收发消息 | [01](course/01-setup.md) |
| 话题（Topic） | 按名字区分的消息通道：发布者写、订阅者读 | [01](course/01-setup.md) |
| 命名空间 | 话题前缀，多机区分飞机：gz_x500 → /x500_0 | [01](course/01-setup.md)、[12](course/12-formation.md) |
| DDS | ROS2 底层发布/订阅中间件标准 | [00](course/00-basics.md) |
| uXRCE-DDS | 嵌入式轻量 DDS，PX4 侧协议；需 Agent 翻译 | [00](course/00-basics.md)、[01](course/01-setup.md) |
| MicroXRCEAgent | PX4 与 ROS2 之间的协议代理（UDP 8888），必需环节 | [01](course/01-setup.md) |
| QoS | 通信可靠性策略；发布端 BEST_EFFORT+VOLATILE，订阅端 BEST_EFFORT+TRANSIENT_LOCAL | [02](course/02-offboard.md) |
| RELIABLE / BEST_EFFORT | 可靠重传 / 尽力而为；两端不匹配会静默收不到或死锁 | [02](course/02-offboard.md) |
| px4_msgs | PX4 消息的 ROS2 定义，版本须与 PX4 配套（1.15） | [01](course/01-setup.md) |
| colcon | ROS2 工作区编译工具 | [01](course/01-setup.md) |
| launch | ROS2 的批量启动描述文件（.launch.py） | [04](course/04-waypoints.md) |
| TF | ROS2 的坐标变换树：各坐标系间的平移旋转关系 | [07](course/07-slam.md) |
| use_sim_time | 让节点用仿真时钟（/clock）而非系统时钟 | [07](course/07-slam.md) |

## 感知与规划
| YOLO | 单阶段目标检测模型，一次前向同时给出框与类别 | [05](course/05-yolo.md) |
| NMS | 非极大值抑制：同一目标多个候选框时只保留最可信的那个 | [05](course/05-yolo.md) |
| letterbox | 保持宽高比缩放到网络输入尺寸，空余部分填灰边 | [05](course/05-yolo.md) |
| CUDA | NVIDIA 的 GPU 计算平台；模型推理可用 GPU 加速 | [05](course/05-yolo.md)、[15](course/15-sim2real.md) |

| 术语 | 白话解释 | 深入 |
|---|---|---|
| 里程计（Odometry） | 随时间估计的位姿+速度（含协方差） | [07](course/07-slam.md) |
| SLAM | 边移动边建图并定位自己 | [07](course/07-slam.md) |
| 栅格地图 | 把地面切成小格，标"占用/空闲"；nav2 格式 pgm+yaml | [07](course/07-slam.md) |
| 点云 | 三维测量点集合；激光/深度相机产出 | [03](course/03-sensors.md) |
| ring / time | 点云的线号与时间戳字段；LIO 需要，gz 点云缺，要补 | [07](course/07-slam.md) |
| ikd-tree | 增量式 k-d 树，LIO 用来快速配准点云；inf 点会破坏它 | [07](course/07-slam.md) |
| LIO | 激光惯性里程计：激光+IMU 融合估计位姿（Point-LIO） | [07](course/07-slam.md) |
| VIO | 视觉惯性里程计：相机+IMU 融合（本课程未成功，研究案例） | [08](course/08-vio.md) |
| ArUco | 黑白方形基准标记，可识别 ID 与位姿；精准降落的靶标 | [06](course/06-precision-landing.md) |
| 视觉伺服 | 用图像误差闭环控制运动（对准靶标） | [06](course/06-precision-landing.md) |
| 势场法 | 目标吸引+障碍/邻居斥力的合成速度控制 | [09](course/09-planning.md)、[13](course/13-swarm.md) |
| 自标定 | 用已知输入+实测输出在线学坐标映射，不靠约定假设 | [09](course/09-planning.md) |
| 行为树 | 用树形结构编排任务：Sequence/Selector/装饰器 | [10](course/10-bt.md) |
| 判分指标 | 可核验阈值（如高度误差<0.4m），grade.py 自动执行 | [16](course/16-grading.md) |
