# 截图与录屏方法

> 本项目所有 README 里的图与 GIF 都是这套流程产出的。踩过的坑集中在三处：
> **抓错窗口**、**录到静止画面**、**GIF 太大**。

---

## 零、先记住一条铁律：抓**窗口**，不要抓整个屏幕

```bash
# ❌ 错误：录全屏。运动会把浏览器/终端一起录进去
ffmpeg -f x11grab -i :0.0 out.mp4

# ✅ 正确：先拿到 Gazebo 窗口 id，只录这个窗口
WID=$(python3 tools/raise_gz_window.py --wait 30 | awk '{print $1}')
```

两个原因：
1. **内容错**：我们踩过一次，生成的 GIF 里全是 GitHub 页面（浏览器在前台）。
2. **X11 限制**：对被遮挡的窗口做 `XGetImage` 会报 `BadMatch` 或抓到黑块 ——
   所以 `raise_gz_window.py` 会先把窗口**置顶**再返回 id。

```bash
# 工具输出示例: "98566158 1200x1000"
python3 tools/raise_gz_window.py --wait 40        # 最多等 40s 直到窗口出现
```

---

## 一、静态截图（Gazebo 窗口）

```bash
export DISPLAY=:0
WID=$(python3 tools/raise_gz_window.py --wait 30 | awk '{print $1}')
import -window "$WID" screenshots/xx_name.png     # ImageMagick
```

- 用于：README 场景图、`docs/SCENARIOS.md` 画廊、问题复现记录
- 本项目截图命名习惯：`screenshots/NN_场景名.png`（如 `20_indoor_hall.png`）

---

## 二、录 Gazebo 窗口（视频）

```bash
export DISPLAY=:0
WID=$(python3 tools/raise_gz_window.py --wait 30 | awk '{print $1}')

ffmpeg -loglevel error -nostdin -y \
  -f x11grab -window_id "$WID" -framerate 12 \
  -i "$DISPLAY" \
  -pix_fmt yuv420p -c:v libx264 -preset ultrafast -crf 28 \
  videos/xxx.mp4 &

# 录完停止（SIGINT 才能让 mp4 正常收尾）
kill -INT %1
```

**参数要点**（都踩过）：
- 必须 `-window_id "$WID" -i "$DISPLAY"`；**不要**写 `-video_size 1x1` 之类的占位（会录出空文件）
- `-framerate 8~12` 足够，太高会让文件暴涨
- **停止要用 `kill -INT`**，`kill -9` 会留下没写完的 mp4（无法播放）

---

## 三、录 ROS 图像话题（检测框这类"不在 Gazebo 窗口里"的画面）

YOLO 的检测框是画在 ROS 话题 `/drone_sim_yolo_detector/annotated` 上的，
**Gazebo 窗口里看不到**，必须直接录话题：

```bash
python3 tools/record_image_topic.py \
  --topic /drone_sim_yolo_detector/annotated \
  --out videos/vision.yolo_detect.mp4 \
  --seconds 40
```

工具会打印实际帧率，例如 `完成: 151 帧 / 40.1 s (3.8 fps 实际)` ——
**必须核对帧数**，`0 帧` 说明话题没数据（节点崩了/话题名错）。

---

## 四、一键"判定 + 录屏"（推荐给验收场景）

```bash
# 通用场景：跑判分的同时录 Gazebo 窗口，并自动镜头跟随
bash scripts/grade_video.sh vision.aruco_precision_landing

# 室内飞行：自带边界自检（越墙/撞顶直接拒绝起飞）
bash scripts/record_indoor_video.sh indoor_hall
SPAWN_X=0 SPAWN_Y=0 CIRCLE_RADIUS=2.0 bash scripts/record_indoor_video.sh indoor_hall
```

这两个脚本已经内置了我们踩过的所有防护：

| 防护 | 作用 |
|---|---|
| `clean_sim.sh` 先清场 | 避免连到上一轮的仿真实例（否则录到别的世界） |
| **等 `armed` 且高度 > 2 m 才开录** | 否则录到的是**地面静止画面**（曾把静止画面当成"飞行演示"） |
| `raise_gz_window.py` + `-window_id` | 只录 Gazebo 窗口 |
| `/gui/follow` 镜头跟随 | 固定机位下无人机会很小甚至出画 |
| 录制前删除旧产物 + `REC_OK` 标记 | 防止"没录到却拿旧视频生成 GIF" |

---

## 五、控制相机取景的三种手段

| 手段 | 命令 / 位置 | 适用 |
|---|---|---|
| 启动默认视角 | 世界文件里的 `<camera_pose>x y z roll pitch yaw</camera_pose>` | 固定机位截图 |
| 运行时移动视角 | `gz service -s /gui/move_to/pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --req 'name:"c", position:{x:0,y:-14,z:7}, orientation:{x:0,y:0.42,z:0,w:0.9}'` | 换角度拍照 |
| 跟随无人机 | `gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --req 'data:"x500_mono_cam_0"'` | **飞行视频必用** |

---

## 六、MP4 → GIF（README 内联播放）

```bash
bash scripts/make_gif.sh videos/xxx.mp4 videos/xxx.gif        # 默认压到约 15s
bash scripts/make_gif.sh videos/xxx.mp4 videos/xxx.gif 20     # 指定目标秒数
```

**体积控制**（README 里超过 2 MB 会明显拖慢加载）：

```bash
ffmpeg -y -i videos/xxx.mp4 \
  -vf "setpts=PTS/3.2,fps=6,scale=360:-2:flags=lanczos,\
split[a][b];[a]palettegen=max_colors=48[p];[b][p]paletteuse=dither=bayer" \
  -loop 0 videos/xxx.gif
```

实测：1200×1000 / 42 s 原片（11 MB）→ 360 宽 / 6 fps / 48 色 → **1.6 MB**。
经验值：**宽度 360–420、fps 6–8、48–64 色、时间压缩 3 倍左右**。

---

## 七、验收：怎么确认"真的录到了东西"

```bash
# 1) 帧数与分辨率
ffprobe -v error -select_streams v -count_frames \
  -show_entries stream=nb_read_frames,width,height -of csv=p=0 videos/xxx.mp4

# 2) 抽一帧看看
ffmpeg -y -i videos/xxx.mp4 -vf "select=eq(n\,250)" -vframes 1 /tmp/check.png

# 3) 判"是不是空白画面"（本项目经验值）
ls -l /tmp/check.png            # 空白帧约 5.7 KB；正常场景 200 KB 以上
```

**判断技巧**：抽两个不同时刻的帧算 MD5，**完全相同 = 画面没动**，
说明飞机没飞（或相机对着墙）。我们就是靠这招发现"录到的其实是地面静止画面"。

另一个更硬的办法：录制脚本里先轮询
`/fmu/out/vehicle_local_position` 的 `z` 与 `/fmu/out/vehicle_status` 的 `arming_state`，
**必须 `armed=2` 且高度 > 2 m 才开录**（`record_indoor_video.sh` / `record_yolo_video.sh` 都已内置）。

---

## 八、踩坑速查表

| 症状 | 根因 | 处置 |
|---|---|---|
| 视频里是浏览器/终端 | 录了全屏 | 用 `-window_id "$WID"` |
| 报 `BadMatch` 或抓到黑块 | 窗口被遮挡 | 先 `raise_gz_window.py` 置顶 |
| 录出 0 KB / 无法播放 | 用了 `kill -9` 或 `-video_size 1x1` | 用 `kill -INT`，参数照抄第二节 |
| 视频是静止画面 | 飞机没起飞就开录 | 等 `armed` + 高度 > 2 m |
| 画面里看不到无人机 | 固定机位 | `/gui/follow` 跟随 |
| YOLO 框在视频里没有 | 录的是 Gazebo 窗口 | 改录 `annotated` 话题 |
| GIF 十几 MB，README 卡 | 分辨率/fps/颜色过高 | 见第六节参数 |
| 帧数 0（`0 帧 / 40s`） | 话题没数据（节点崩/话题名错） | 先 `ros2 topic list` + 看节点日志 |

---

**相关文档**：[环境资产指南 ENVIRONMENTS](ENVIRONMENTS.md) · [场景画廊 SCENARIOS](SCENARIOS.md) ·
[运维手册 OPERATIONS](OPERATIONS.md)
