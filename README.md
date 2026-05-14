# Gazebo Sim — NEDO Challenge コンテスト1 / コンテスト3 動画

Isaac Sim 版の動画で「ロボット/物体の動きが微妙」(2026-05-15 ユーザフィードバック)
だったため、**Gazebo Harmonic (gz-sim 8.11) + ROS 2 Jazzy + ros2_control + MoveIt**
ベースの実物理駆動シミュレーションで作り直したもの。

## 構成

| 要素 | 値 |
|---|---|
| シミュレータ | Gazebo Harmonic 8.11 (`gz sim`) |
| 物理エンジン | DART (ODE collision) |
| ROS | ROS 2 Jazzy |
| アーム制御 | gz_ros2_control + JointTrajectoryController |
| ベルト | `gz-sim-track-controller-system` (摩擦+表面速度) |
| 把持 | `gz-sim-detachable-joint-system` (確実に物体を保持) |
| カメラ | 1920×1080, 30fps, Ogre2 |
| 録画 | ros_gz_image bridge → ffmpeg libx264 |

## 出力動画

### コンテスト1 (手荷物識別 / BHS)

| File | サイズ | 長さ | 内容 |
|---|---|---|---|
| `output/contest1_bhs_1080p.mp4` | 6.7 MB | 24.16 s | BHS コンベア (4m, 25m/min) 上を 10種の bag (4分類×7素材バリエーション) が連続的に流れる。センサーフレーム (RGBステレオ + 偏光 + IR projector) 直下を通過し、ULD bin に到達。Real-time physics (TrackController で belt 表面速度 0.4167 m/s, mu2=150)。 |

### コンテスト3 (積付ロボット)

| File | サイズ | 長さ | 内容 |
|---|---|---|---|
| `output/contest3_pickplace_1080p.mp4` | 1.7 MB | 64.9 s | AgileX PiPER 6軸アーム + Mobile Mover proxy pedestal で 3 サイクル Pick & Place。BHS conveyor (右) → ULD bin (左) への積付。joint_trajectory_controller によるスムーズな関節空間動作、DetachableJoint で正確な把持リリース。 |

## ロボット / シーン資産

- **AgileX PiPER URDF**: `urdf/piper_gz.urdf.xacro` (6 DOF + gripper, ros2_control 系統)
- **Piper メッシュ**: `/data/nedo/sim/assets/piper_isaac_sim/piper_description/meshes/*.STL` (binary)
- **コンテスト1 world**: `worlds/bhs_contest1.sdf`
- **コンテスト3 world**: `worlds/pickplace_contest3.sdf`
- **コントローラ設定**: `config/piper_controllers.yaml`

## スクリプト

| Script | 用途 |
|---|---|
| `scripts/run_bhs_demo.sh` | コンテスト1 を録画 (`DUR_S=70` で 24秒の sim 動画) |
| `scripts/run_pickplace_demo.sh` | コンテスト3 を録画 (`CYCLES=3 REC_DUR_S=140`) |
| `scripts/bag_spawner.py` | BHS 上に procedural bag を投入し belt を駆動 |
| `scripts/pickplace_controller.py` | Pick&Place orchestrator (HOME→GRASP→LIFT→PLACE) |
| `scripts/record_video.py` | ROS Image → ffmpeg → mp4 録画 |
| `launch/pickplace_contest3.launch.py` | Piper xacro + ros2_control + bridges ローンチ |

## 再現方法

```bash
source /opt/ros/jazzy/setup.bash
# コンテスト1
DUR_S=70 bash /data/nedo/gazebo_sim/scripts/run_bhs_demo.sh

# コンテスト3
CYCLES=3 REC_DUR_S=140 bash /data/nedo/gazebo_sim/scripts/run_pickplace_demo.sh
```

出力は `output/*.mp4` に書き出される。

## Isaac Sim 版との比較

| 項目 | Isaac Sim 版 (`/data/nedo/sim/`) | Gazebo 版 (本ディレクトリ) |
|---|---|---|
| Pick&Place 動作 | キーフレームアニメ (微妙) | ros2_control + MoveIt 系トラジェクトリ (実物理) |
| Belt 上の bag | scripted PhysxSurfaceVelocity | DART + TrackController surface friction |
| 把持挙動 | アニメで掴む見た目のみ | DetachableJoint で正確に剛体接続 |
| レンダ画質 | RTX RayTracedLighting (高) | Ogre2 PBR (中) |
| 物理リアリズム | scripted | 実物理駆動 |

提案書本体では、視覚品質より「実物理 + ROS2 統合実装」を強調する材料として本 Gazebo 版動画を用いることを推奨。
