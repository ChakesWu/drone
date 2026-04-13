# ArduPilot 飞控配置说明

> **必须在 Mission Planner 中完成以下参数设置，才能让自主飞行正常工作。**
> 软件代码无法自动完成这些配置。

---

## 背景

目前系统使用 Cartographer + YDLidar T-MINI 进行激光雷达 SLAM 定位。  
定位结果通过 `cartographer_vision_bridge.py` 桥接到 MAVROS 的 `/mavros/vision_pose/pose` 话题。  
飞控 EKF（EKF3）需要被告知使用这个外部视觉定位源，否则即使桥接节点在正常发送数据，飞控也不会采用。

---

## 必须设置的参数

在 Mission Planner → Full Parameter List 中搜索并修改以下参数：

| 参数 | 设置值 | 说明 |
|---|---|---|
| `AHRS_EKF_TYPE` | `3` | 切换到 EKF3（支持外部定位源） |
| `EK3_ENABLE` | `1` | 启用 EKF3 |
| `EK3_SRC1_POSXY` | `6` | XY 位置源 = 外部导航 (ExtNav) |
| `EK3_SRC1_POSZ` | `1` | Z 位置源 = 气压计（稳定，推荐室内使用） |
| `EK3_SRC1_YAW` | `6` | Yaw（航向角）源 = 外部导航 |
| `VISO_TYPE` | `1` | 启用视觉里程计输入接口 |

> **设置完成后必须重启飞控（断电重连）才能生效。**

---

## 验证配置是否生效

按照以下启动顺序运行系统，然后验证：

```bash
# 终端 1：MAVROS
ros2 launch mavros apm.launch fcu_url:="serial:///dev/ttyACM0:57600"

# 终端 2：激光雷达
ros2 launch ydlidar_ros2_driver Tmini_launch.py

# 终端 3：Cartographer 建图
ros2 launch tmini_cartographer_py tmini_cartographer.launch.py

# 终端 4：桥接节点（核心修复）
python3 ~/DroneWS/cartographer_vision_bridge.py
```

然后验证：

```bash
# 1. 确认桥接节点在发数据（应有连续 PoseStamped 消息）
ros2 topic echo /mavros/vision_pose/pose

# 2. 确认飞控 EKF 已融合（应有连续 PoseStamped 消息）
ros2 topic echo /mavros/local_position/pose

# 3. 查看 TF 是否正常（应输出坐标数字）
ros2 run tf2_ros tf2_echo map base_link
```

如果第 2 条命令有持续输出，说明配置成功，可以运行飞行任务：

```bash
# 终端 5：飞行任务
python3 ~/DroneWS/drone_cartographer_control.py
```

---

## 常见问题

**Q: `/mavros/vision_pose/pose` 有数据，但 `/mavros/local_position/pose` 没有？**  
A: 飞控参数未设置或未重启飞控。检查 `EK3_SRC1_POSXY=6` 和 `VISO_TYPE=1`。

**Q: Cartographer 建图漂移很厉害？**  
A: 确保激光雷达已固定在机体上，不要在飞行中晃动。初始化时让无人机静止 10 秒让 Cartographer 建立初始地图。

**Q: 飞控切换 GUIDED 后无人机不动？**  
A: 检查 `drone_cartographer_control.py` 启动日志，确认出现 `✅ 飞控 EKF 已就绪` 的提示。

---

## 相关文件

| 文件 | 说明 |
|---|---|
| `cartographer_vision_bridge.py` | 【新增】Cartographer TF → MAVROS vision_pose 桥接节点 |
| `drone_cartographer_control.py` | 【修改】加入 EKF 就绪等待逻辑 |
| `SETUP_ARDUPILOT.md` | 【新增】本文档，飞控参数配置说明 |
