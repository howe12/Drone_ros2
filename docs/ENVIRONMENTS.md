# 虚拟环境（世界）资产指南：怎么找、怎么选、怎么造

> 面向**课程与环境制作者**。本文的每条结论都来自本项目实际踩过的坑，附实测证据。
> 如果你想给某个实验找场景，先看第五节的课程配对表；
> 如果你要自己加一个新世界，直接看第三节的 SOP 检查表。

---

## 一、先建立一个基本认识

一个"仿真世界"（world）在 Gazebo Harmonic 里只是一个 `.sdf` 文件，它本身几乎不包含几何，
而是**引用模型**（model）。所以环境质量问题几乎都出在"引用的东西"上：

```
world.sdf
├── 插件（物理/传感器/IMU/GPS…）      ← 缺一个就可能"能加载但飞不起来"
├── 光照与天空
├── 地面 / 墙体（可以内联几何）        ← 单面网格会导致"从里面看是空的"
└── include: model://xxx             ← 三种来源，决定能不能离线跑
    ├── 本地模型（仓库/third_party 里）      ✅ 最稳
    ├── Fuel 在线模型（首次运行下载）        ⚠️ 有网才行，材质还可能是绝对 URL
    └── Gazebo Classic 老资产              ❌ Harmonic 下多半崩溃
```

**核心判断**：一个世界能不能用，不看它"能不能加载"，而看这四件事：
1. 模型是否**自包含**（不联网也能起）
2. 机载相机**看得到东西**（不是纯色背景）
3. 无人机**能解锁起飞**（插件齐、EKF 有数据）
4. **边界安全**（出生点、航线不撞墙）

---

## 二、去哪找现成的：四个可靠来源

| 来源 | 有什么 | 优劣 | 适用 |
|---|---|---|---|
| **Gazebo Fuel**<br>`app.gazebosim.org` | 上万模型与世界：`baylands`(393 MB)、`Warehouse`、`Apartment`、家具（货架/桌子/椅子/纸箱/冰箱）、`person_standing`、车辆… | ✅ 数量最多、带贴图<br>⚠️ 体积大、部分模型材质用 **https 绝对地址**（断网就变黑）、建筑常是**单面墙** | 找**道具**很好用；直接拿**建筑**当室内场景要谨慎 |
| **PX4 自带世界**<br>`PX4-Autopilot/Tools/simulation/gz/worlds` | `baylands` `default` `lawn` `windy` | ✅ 与 PX4 版本配套、验证充分、**全本地** | 控制类实验的默认选择 |
| **XTDrone2 世界**<br>`third_party/XTDrone2/xtd2_gz_sim/worlds` | 8 个：`default` `aruco` `walls` `lawn` `windy` `rover` `baylands` `indoor1` | ✅ 已配好桥接与机型<br>⚠️ `baylands` 需联网、`indoor1` 依赖 Classic 资产会崩 | 感知/避障/SLAM 的起点 |
| **本项目自建**<br>`patches/0016/0019/0020` | `e7_obstacles`（障碍群）、`yolo_targets`（人/车）、`indoor_hall`（20×20 室内大厅，36 件家具） | ✅ 全本地、边界自检、已验证 | 课程专用场景 |

> 其他著名资源（未在本仓库验证，供探索）：AWS RoboMaker 的 `Small House` / `Bookstore`（室内家居、
> 物品丰富），以及 Gazebo 社区的仓库货架类世界。引入前请按第三节 SOP 走一遍。

---

## 三、自己造：SOP 检查表

> 这是我们做 `indoor_hall` 时总结的完整流程，每一步都有"不这么做会怎样"的教训。

### 步骤 1：判断候选是否值得用（5 分钟，能省 1 小时）

```bash
# ① 它会不会联网？模型 SDF 里出现 https:// 就是绝对地址（断网会渲染成黑色）
grep -c "https://fuel" <model>/model.sdf

# ② 它有多大？（>100 MB 要考虑下载与渲染开销）
du -sh <model>

# ③ 它的真实尺寸与单位？（Collada 常见 <unit meter="0.0254"/>，即英寸）
grep -oE '<unit[^>]*>' <model>/meshes/*.dae | head -1
```

**教训**：`baylands` 的 10 处 URI 是 `https://fuel.gazebosim.org/...`，于是每次加载都联网，
网络受限时贴图取不到 → 树变成**纯黑剪影**。可跑 `scripts/fix_fuel_offline.sh` 改写为相对路径。

**教训**：`Apartment` 的 DAE 是**英寸**单位（包围盒 797×877×583 → 实际 20×22×15 m）。
不换算就会把出生点算错。

### 步骤 2：装模型，并核对**目录名**

gz 用 `model://<目录名>` 解析，而模型自己的 `model.sdf` 里也常写 `model://<同名>/meshes/...`：

```bash
# 看模型内部怎么引用自己
grep -oE "model://[^/\"<]+" <model>/model.sdf | head -3
```

**目录名必须与内部引用**完全一致（**大小写敏感**：`SquareShelf` ≠ `squareshelf`）。

**教训**：我批量重命名目录时正则写错（把 `model://x500_base</uri>` 也当成引用），
误把 8 个机型目录改名，直接破坏了整机模型。**批量改名后必须逐个核对关键目录存在**。

### 步骤 3：写世界文件 —— 插件清单别漏

```xml
<plugin filename="gz-sim-physics-system"     name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<plugin filename="gz-sim-contact-system"     name="gz::sim::systems::Contact"/>
<plugin filename="gz-sim-imu-system"         name="gz::sim::systems::Imu"/>          <!-- 缺它不能解锁 -->
<plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/> <!-- 气压计 -->
<plugin filename="gz-sim-navsat-system"      name="gz::sim::systems::NavSat"/>        <!-- GPS -->
<plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>
<plugin filename="gz-sim-sensors-system"     name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

**教训**：我第一版世界只写了 4 个插件，PX4 报 `Preflight Fail: Gyro Sensor 0 missing` /
`Compass Sensor 0 missing`，**无人机永远不能解锁**。补上 `Imu` 插件后立刻正常。

### 步骤 4：无头快跑，抓解析错误（最快的反馈环）

```bash
export GZ_SIM_RESOURCE_PATH="$PWD/third_party/XTDrone2/xtd2_gz_sim/models"
gz sim -s -r "$PWD/ros2_ws/install/xtd2_gz_sim/share/xtd2_gz_sim/worlds/<world>.sdf" 2>&1 \
  | grep -E "could not be resolved|Error" | head
```

这一步几秒就能暴露"模型名大小写错了""install 里没有新模型"等全部问题。
**不要**直接起整套 PX4 栈去试 —— 那样一轮要 90 秒，而且错误被埋在几百行日志里。

> 注意：新增模型目录后必须 `colcon build --packages-select xtd2_gz_sim`，
> 否则 `install/share` 里没有它（`--symlink-install` 是**逐文件**建链，不是整个目录）。

### 步骤 5：定出生点 + 边界自检

```bash
SPAWN_X=0 SPAWN_Y=0 SPAWN_Z=0.3 bash scripts/run_simulation.sh xtd2 gz_x500 indoor_hall
```

**教训**：出生点必须避开几何；更要紧的是**轨迹**也要在边界内。
`offboard_demo` 的**画圆圆心就是起飞点** —— 我们用 `(-4,-9)` + 半径 2 m，圆周到 `y=-11`，
而墙内表面在 `y=-9.85`，**实测撞墙**。

现在 `scripts/record_indoor_video.sh` 内置自检，越界直接以退出码 3 拒绝起飞：

```
圆周外包:   x ≤ 6.00 m, y ≤ 11.00 m
✗ 圆周在 y 方向越墙 1.15 m
✗ 边界自检未通过，拒绝起飞（避免撞墙/撞顶）。
```

### 步骤 6：家具要静态

Fuel 道具默认是**动态刚体**，被撞或落地震动会倒。

**教训**：第一版 `indoor_hall` 录制出来**货架倒了一地**。给所有 include 加
`<static>true</static>` 后恢复正常（本项目 42 处）。

### 步骤 7：验证"机载相机真的有画面"——不要用 GUI 截图判断

```bash
ros2 run drone_sim ...   # 起栈后
python3 tools/record_image_topic.py --topic /x500_mono_cam_0/camera/image_raw \
        --out /tmp/v.mp4 --seconds 3
ffmpeg -y -i /tmp/v.mp4 -vframes 1 /tmp/v.png     # 看这一帧
```

**判断技巧**：如果帧文件小得可疑（本项目里空白帧 = **5762 字节**），
或两帧**哈希完全相同**，那就是"什么都看不到"，不是"画面干净"。

**教训**：`Warehouse` 和 `Apartment` 在 GUI 里看着挺好，但无人机进去后
**机载相机输出纯色空白** —— 因为这类 Fuel 建筑的墙是**单面网格**，从内部看直接穿透。
我就是先用 GUI 截图判断"加载成功"，才发现这个问题的。

### 步骤 8：固化与文档化

- 世界文件放进 `patches/00XX-*.patch`（`third_party` 不入库），`setup_workspace.sh` 追加一行
- 模型资产写进 `scripts/fetch_fuel_models.sh`（按需下载，不入库）
- 在 `docs/SCENARIOS.md` 记录：内容、来源、许可、实测状态（包括失败项）
- 录屏用 `scripts/record_indoor_video.sh` 或 `scripts/grade_video.sh`（只录 Gazebo 窗口 + 镜头跟随）

---

## 四、三档策略：按投入选路线

| 档位 | 做法 | 投入 | 适合 |
|---|---|---|---|
| **① 直接用** | 用上游世界的现成配置（`default`/`walls`/`lawn`/`windy`/`aruco`）| 10 分钟 | 控制类实验；先跑通再谈场景 |
| **② 组合**（推荐） | **可信底座 + Fuel 道具**：地面/墙自建（双面箱体），道具从 Fuel 挑（桌椅/货箱/货架/人/车）| 1–2 小时 | 本项目 `yolo_targets`、`indoor_hall` 都是这么来的 |
| **③ 导入** | 把外部 DAE/OBJ/建筑整体引入（Fuel 建筑、Sketchfab 转换）| 半天+ | 高保真需求；必须先过第三节 SOP |

**为什么推荐②**：底座由我们控制（双面墙、边界已知、插件齐全），道具数量与位置随意调整，
既满足"物体多"又不引入不可控资产。`indoor_hall` 就是这个思路的成品。

---

## 五、课程怎么配环境（按实验类型查表）

| 实验类型 | 推荐环境 | 理由 | 教学注意 |
|---|---|---|---|
| 控制入门（起飞/悬停/航点）| `default` 或 `lawn` | 空旷无干扰，失败原因单一 | 先把"能飞"和"环境无关"分开 |
| 抗风鲁棒性 | `windy`（真实 10 m/s 三轴风场）| SDF 里配了 `<wind>` 与 `enable_wind` | 对比实验：同参数开/关风 |
| 视觉目标检测 | `yolo_targets`（3 车 + 3 行人）| 有明确的 COCO 类别目标 | 相机 99.7° FOV、桨叶会入画（见下）|
| 视觉精准降落 | `aruco` | 自带靶标，贴图已换为 DICT_4X4_50 | 光照对检测有影响 |
| 2D 激光 SLAM | `walls` | 结构化墙线，便于对比地图 | 需 `odom_to_tf` + 里程计插件 |
| 3D 激光里程计 | `baylands` 或 `indoor_hall` | 有足够结构 | `baylands` 需先 `fix_fuel_offline.sh` |
| 局部避障 | `e7_obstacles` | 障碍已知、可算最近距离 | 判定看"最近接近距离"与"障碍距离"|
| 室内飞行 / 室内 SLAM | `indoor_hall` | 20×20 m、36 件家具、有天花板与吊灯 | 需注意"画圆高度下沉"待查项 |

### 环境复杂度阶梯（建议按这个顺序排课）

```
default（空场）
  → walls（有墙）
    → e7_obstacles（有障碍物）
      → indoor_hall（有屋顶、物体多、空间有限）
        → baylands（大场景、远距离、需联网资产）
```

每升一级，学生要处理的新变量：墙/边界 → 障碍与避让 → 室内 GPS 变差与高度估计 →
大规模场景与资产依赖。

---

## 六、把"造环境"设计成学生作业

这是很有价值的训练 —— 学生造环境时会遇到本文所有坑。建议作业要求：

**题目**：为你自己的任务新建一个可复现的仿真世界

| 验收标准 | 检查方法 |
|---|---|
| ① 世界能在无 GUI 下加载无报错 | `gz sim -s -r world.sdf` 无 `could not be resolved` |
| ② 机载相机有真实画面（非纯色）| 取一帧，文件大小正常、与另一时刻**哈希不同** |
| ③ 无人机能解锁并起飞到指定高度 | `armed=2` 且实测高度误差 < 0.3 m |
| ④ 边界安全 | 提供自检脚本或计算过程，证明航线离墙 ≥ 1 m |
| ⑤ 资产可复现 | 模型来源写进 `fetch_*.sh`，世界写成 patch，别人能一键重建 |
| ⑥ 文档齐全 | 内容/来源/许可/实测结果（含失败项）写入 `docs/SCENARIOS.md` |

> 提示学生：**"能加载"≠"能用"**。第②③④三条才是真正的门槛；
> 而且**不要用 GUI 截图**判断②，要用机载相机取帧。

---

## 七、踩坑速查表（都是本项目真实发生过的）

| 症状 | 根因 | 处置 |
|---|---|---|
| 树/建筑渲染成**纯黑** | 模型 SDF 用 https 绝对 URI，断网取不到贴图 | `scripts/fix_fuel_offline.sh` 改本地相对路径 |
| 机载相机**纯色空白**（帧文件 ~5.7 KB）| 建筑是**单面墙**，从内部穿透；或相机埋在实体里 | 改用双面箱体自建，或把相机移到开阔处 |
| `Preflight Fail: Gyro/Compass Sensor 0 missing` | 世界缺 `Imu` 插件 | 补齐插件清单（见步骤 3）|
| `Parser ... uri [model://X/...] could not be resolved` | 目录名与内部引用大小写不一致 / 新模型没重新 build | 统一命名 + `colcon build --packages-select xtd2_gz_sim` |
| 家具**倒地**、被撞飞 | Fuel 道具是动态刚体 | include 加 `<static>true</static>` |
| 起飞/画圆**撞墙** | 画圆圆心 = 起飞点，半径过大 | 出生点居中 + 边界自检（退出码 3）|
| 节点启动即崩，报参数类型错 | CLI 传 `30` 被推断为 INTEGER，而声明是 DOUBLE | 传 `30.0` 浮点字面量 |
| `indoor1` 一加载 gz 就崩 | 引用 Gazebo **Classic** 资产（`cafe`/`coke_can`…）| 别用；用 Harmonic 原生资产重建 |
| 录制视频是黑屏/录到浏览器 | 抓错窗口 | `x11grab -window_id $WID -i $DISPLAY` + 先 raise 窗口 |

---

## 八、许可与致谢（教学也要守规矩）

- Fuel 模型**各有各的许可**（CC0 / CC-BY / 自定义），**教学与公开发布都要致谢**；
  商用前逐个确认，本项目在 [README 致谢章节](../README.md#致谢与引用) 列了已核对的清单。
- 上游平台（PX4 / XTDrone2 / Gazebo / ros_gz）的许可与引用方式同样见该章节。
- 课程若分发镜像或二进制，**GPL/AGPL 组件**（Point-LIO、VINS-Fusion、Ultralytics）的义务需自行评估。

---

**相关文档**：[截图与录屏 RECORDING](RECORDING.md) · [场景画廊 SCENARIOS](SCENARIOS.md) · [平台契约 PLATFORM_API](PLATFORM_API.md) ·
[运维手册 OPERATIONS](OPERATIONS.md) · [平台路线 ROADMAP](ROADMAP.md)
