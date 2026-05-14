# nedo_demo — Argus BHS / Pick & Place 開発基盤

NEDO Challenge（空港グランドハンドリング）コンテスト1「手荷物識別」およびコンテスト3「積付ロボット」の応募者が、本開発で実利用するシミュレーション・統合基盤として構築したリポジトリ。

ROS 2 Jazzy + Gazebo Sim 8（gz-sim）上に、

- 25 m/min 連続走行 BHS コンベア
- マルチモーダル識別装置「Argus」のシーン上モック
- 6軸協働アーム（AgileX PiPER）＋ 移動マニピュレータの URDF
- `ros2_control` を介した実物理関節軌道制御
- 識別結果ライブ可視化 HUD パイプライン

を再現可能な形で実装している。本リポジトリはコンテスト採択後の Stage1 開発期間（2026/7〜2027/2）にそのまま開発リポジトリとして拡張する。

応募者: 安河内 竜二（個人応募）
リポジトリ: https://github.com/Ryujiyasu/nedo_demo

---

## 1. 提供する開発基盤

| レイヤ | 内容 | 本リポジトリ上の実装 | Stage1/2 で拡張する点 |
|---|---|---|---|
| シミュレーション環境 | BHS / メイクエリア / ULD / 移動マニピュレータ / 協働アーム の物理シーン | `worlds/bhs_contest1.sdf`, `worlds/pickplace_contest3.sdf` | 佐賀空港試験ラインの実測値で寸法・配置を校正、LD3-AKE 実寸 ULD、多 ULD 並列 |
| ロボット記述 | AgileX PiPER 6DoF + gripper + 移動マニピュレータベース | `urdf/piper_gz.urdf.xacro`（`ros2_control` ハードウェアインタフェース付き） | 22kg 級ヘビーデューティアーム（AUBO i20 / 安川 GP12 候補）への乗せ替え、マルチモーダルエンドエフェクタ |
| 動作制御 | `joint_trajectory_controller` による関節空間トラジェクトリ実行 | `config/piper_controllers.yaml`, `scripts/pickplace_controller.py` | MoveIt 2 統合（OMPL / CHOMP）、BehaviorTree によるエラーリカバリ、Cartesian path planning |
| 識別 | コンベア上 bag の 4分類 × 7素材 × 寸法計測 のシーン上模擬 | `scripts/bag_spawner.py`（procedural bag 生成、`classify()` で分類タグ付与） | 実機推論パイプライン（YOLO + DINOv2 embedding + 世界手荷物寸法 DB）を本シーンで合成データ学習・評価する |
| 積付計算 | （Stage1 で QUBO 統合予定） | — | QUBO ベース 3D bin-packing ソルバを ULD 内配置・順序決定に転用 |
| 可視化／録画 | Argus 風 HUD（識別フィード）と Pick & Place HUD を OpenCV で重畳しつつ録画 | `scripts/bhs_annotator.py`, `scripts/pickplace_annotator.py`, `scripts/record_video.py` | HUD を実機運用時のオペレータ画面（Web UI）にそのまま転用 |
| 学習データ生成 | Domain Randomization 用シーン | 物理リアリズム検証は本 Gazebo 側、レンダ品質はレイトレース系シミュレータ側で並列構築 | DR pipeline、SKU 別マテリアル DB |

---

## 2. 動画成果物

提案書 §g（類似実績）に添付する動画。

### 2.1 コンテスト1 — Argus BHS 識別パイプライン

**`output/contest1_bhs_1080p.mp4`** — 1920×1080 / H.264 / 50 秒 / 7.3 MB

BHS コンベア（4 m × 0.6 m、25 m/min = 0.4167 m/s 連続走行）上を、4分類 × 7素材 をカバーする 10 種類の手荷物プロシージャル生成サンプルが流れる。識別装置（架台＋RGB ステレオ × 2 ＋ 偏光 × 1 ＋ Active IR projector）直下を通過する各 bag に対し、Argus HUD が以下をライブ出力：

- 個体 ID
- 4分類（H / HX / S / SX）
- 7素材（PLA / MET / FAB / LEA / VIN / PAP / MIX）
- 包絡直方体寸法（mm 単位）
- 世界手荷物寸法 DB の照合ブランド名（Samsonite / Rimowa / Tumi / Travelpro / Briggs & Riley 等）
- 照合スコア
- 推定質量
- 直近 5 件の検出履歴ローリングフィード（BHS PLC / BSM 出力相当）

ベルト挙動は `gz-sim-track-controller-system` プラグインによる表面摩擦駆動（ODE friction direction、mu2 = 150、fdir1 = 0 1 0）。bag は剛体物理シミュレーションで押されるため、停止・加速・スリップ挙動がそのまま再現される。HUD は `bhs_annotator.py` が `/cinematic` トピックを購読し、OpenCV で重畳した上で `/cinematic_annotated` として再パブリッシュ、`record_video.py` が H.264 fragmented mp4 として書き出している。

### 2.2 コンテスト3 — 移動マニピュレータ Pick & Place スケルトン

**`output/contest3_pickplace_1080p.mp4`** — 1920×1080 / H.264 / 25 秒 / 2.4 MB

AgileX PiPER 6 軸協働アーム + 移動マニピュレータベース proxy + 短尺 BHS 出口コンベア + ULD-LD3 風 bin の 4 サイクル積付動作：

```
HOME → PREGRASP_OVER_BELT → GRASP_AT_BELT → SPAWN+ATTACH
       → LIFT_FROM_BELT → PREPLACE_OVER_ULD → PLACE_INTO_ULD
       → DETACH → LIFT_FROM_ULD → HOME → ...
```

各サイクルで `joint_trajectory_controller` がアームを目標関節姿勢へ滑らかに駆動し（`gz_ros2_control` の `GazeboSimSystem` ハードウェアインタフェース経由）、`gz-sim-detachable-joint-system` がグリッパ wrist (`link6`) と bag 剛体を物理的に剛結合する。HUD はサイクル進捗・現在 bag 種別・Phase・ULD 充填台帳（積み済み bag リスト + 累積質量）を表示する。

本デモの bag サイズは PiPER の 4 cm グリッパストロークに合わせた 10 cm キューブ。実機 22-32 kg 級フルサイズ手荷物への対応は Stage2 で 22 kg 級アーム（AUBO i20 / 安川 GP12 等）への乗せ替えと、マルチモーダルエンドエフェクタ（吸着・すくい上げ・取っ手把持）の実装によって達成する。

---

## 3. アーキテクチャ

```
┌──────────────────────────────────────────────────────────────────┐
│ ROS 2 Jazzy                                                      │
│                                                                  │
│  ┌─────────────────┐  ┌────────────────┐  ┌──────────────────┐   │
│  │ pickplace_      │  │ bag_spawner    │  │ {bhs,pp}_        │   │
│  │ controller.py   │  │                │  │ annotator.py     │   │
│  │ ・arm trajectory│  │ ・belt drive   │  │ ・Argus HUD      │   │
│  │ ・bag spawn     │  │ ・bag generate │  │ ・event ingest   │   │
│  │ ・attach/detach │  │ ・event emit   │  │ ・OpenCV overlay │   │
│  └────────┬────────┘  └────────┬───────┘  └────────┬─────────┘   │
│           │ /arm_controller/   │ gz service        │             │
│           │ joint_trajectory   │ /world/.../create │             │
│           ▼                    ▼                   │             │
│  ┌────────────────────────────────────────┐       │             │
│  │ controller_manager (gz_ros2_control)   │       │             │
│  │  ・JointTrajectoryController × 2       │       │             │
│  │  ・JointStateBroadcaster               │       │             │
│  └────────────────┬───────────────────────┘       │             │
│                   │                                │             │
│  ┌────────────────▼────────────────┐  ┌────────────▼─────────┐  │
│  │ gz_ros2_control system plugin   │  │ ros_gz_image bridge   │  │
│  │ (embedded inside gz sim)        │  │ /cinematic →          │  │
│  │ ・reads URDF <ros2_control>     │  │ sensor_msgs/Image     │  │
│  └────────────────┬────────────────┘  └────────────┬──────────┘  │
└───────────────────┼─────────────────────────────────┼─────────────┘
                    │ gz transport                    │
                    ▼                                 │
┌──────────────────────────────────────────────────────┼────────────┐
│ Gazebo Sim 8.11                                      │            │
│                                                      │            │
│ Physics (DART + ODE) ◀ TrackController plugin        │            │
│                       ◀ DetachableJoint plugin       │            │
│                       ◀ Sensors plugin (Ogre2)       │            │
│                                                      │            │
│ Worlds:                                              │            │
│   ・bhs_world (Argus 識別ライン)                      │            │
│   ・pickplace_world (Pick & Place セル)               │            │
│                                                      │            │
│ Models:                                              │            │
│   ・piper (PiPER 6DoF + gripper + base pedestal)      │            │
│   ・bhs_conveyor (kinematic belt + side rails)        │            │
│   ・sensor_frame (RGB stereo + Pol + IR projector)    │            │
│   ・uld_ld3 (ULD container shell)                     │            │
│   ・bags (dynamic, runtime-spawned)                   │            │
│   ・cinematic_cam (sensor → ROS image) ──────────────┘            │
│                                                                   │
└───────────────────────────────────────────────────────────────────┘
                    │
                    ▼ /cinematic_annotated
             ┌──────────────────┐
             │ record_video.py  │ → ffmpeg → fragmented mp4
             │ /cinematic_ann → │   (frag_keyframe+empty_moov、
             │ raw RGB pipe     │    途中切断にも耐える）
             └──────────────────┘
```

主要な技術判断:

| 課題 | 採用解 | 棄却した代替と理由 |
|---|---|---|
| BHS ベルト上 bag 駆動 | `gz-sim-track-controller-system` で ODE 摩擦方向（mu2 = 150、fdir1 = 0 1 0）に表面速度コマンドを与える | (a) bag 個別の `set_velocity` サブプロセス呼出 — 30Hz update 不可、(b) kinematic pose-sync — 物理衝突が反映されない |
| グリッパ把持 | bag SDF 側の `gz-sim-detachable-joint-system` プラグインで wrist link (`link6`) と bag body を剛体接続。spawn 位置は `gz model -m piper` の link6 ワールド姿勢クエリで動的決定 | 摩擦のみによる物理把持 — PiPER 4 cm ストロークでは滑り出す。`gripper_base` は URDF→SDF 変換時の fixed-joint コラプスで消滅する |
| URDF mimic joint | `gripper_right` を fixed joint 化。把持は DetachableJoint で代替 | URDF `<mimic>` を残すと `gz_ros2_control` が "Activated mimic joints cannot have command interfaces" で起動失敗 |
| 動画録画 | `record_video.py`（ROS Image → ffmpeg pipe）で `-movflags +frag_keyframe+empty_moov+default_base_moof` の fragmented mp4 を出力 | gz-sim 内蔵 `CameraVideoRecorder` プラグイン — 強制終了時に moov atom が未確定で再生不可になる事故が頻発 |
| HUD 重畳 | OpenCV ノード（JSONL イベント購読）が `/cinematic` → `/cinematic_annotated` を再配信 | (a) gz-sim GUI overlay — ヘッドレス録画不可、(b) ffmpeg drawtext — 動的レイアウト対応不可 |
| rclpy + subprocess 干渉 | `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)` | デフォルト init は `subprocess.run(["gz", ...])` の SIGINT 伝搬で context が即 invalid になる |

---

## 4. 識別アルゴリズムの開発スタックとの接続

本リポジトリは識別装置（Argus）のシーン上モックを提供する。実機推論パイプラインとの接続は以下の経路で行う:

```
                    ┌──────────────────────────┐
                    │ Argus 学習データ生成      │
                    │ （物理リアリズム検証は    │
                    │   本リポジトリで実施）    │
                    └────────┬─────────────────┘
                             │ synthetic RGB / depth / seg / bbox
                             ▼
                    ┌──────────────────────────┐
                    │ YOLO + DINOv2 embedding  │
                    │ + 世界手荷物寸法 DB 検索  │
                    └────────┬─────────────────┘
                             │ (label, dims, sku_match)
                             ▼
              ┌──────────────────────────────────────┐
              │ ROS2 トピック /argus/detection (実機) │
              │ もしくは /tmp/bag_events.jsonl (シム)  │
              └────────┬─────────────────────────────┘
                       │
                       ▼
              ┌──────────────────────────────┐
              │ bhs_annotator.py / 実機 HMI  │
              │ → BHS PLC 連携 → BSM 出力    │
              └──────────────────────────────┘
```

本リポジトリ内では bag spawn と同時に `classify()` 関数が分類タグを JSONL イベントとして出力する模擬実装になっている。実機推論パイプラインを差し込む箇所は `bag_events.jsonl` の入力経路 1 箇所のみで、`bhs_annotator.py` 以下の HUD・記録系は変更なく動作する。

---

## 5. 積付アルゴリズム（QUBO）統合計画

QUBO 定式化による 3D bin-packing を本リポジトリの Pick & Place 系に統合する。

```
   識別装置出力（bag 種別 + 寸法 + 質量）
         │
         ▼
   ┌──────────────────────────┐
   │ QUBO 定式化               │
   │ ・配置位置 (x, y, z)     │
   │ ・配置順序                │
   │ ・回転（向き）            │
   │ Constraints:              │
   │   ・荷重制約              │
   │   ・重心制約              │
   │   ・破損防止 (柔/硬上下)  │
   │   ・取っ手・タグ向き       │
   └─────────┬────────────────┘
             │ Annealing
             ▼
   ┌──────────────────────────┐
   │ 配置プラン                │
   │ → pickplace_controller.py│
   │   ・arm waypoints        │
   │   ・detach position      │
   │ → 群協調 (Stage2)        │
   └──────────────────────────┘
```

Stage1 では単機 + 単 ULD で QUBO 結果を実シーン上に配置（充填率・荷崩れ評価）、Stage2 で多機 × 多 ULD の並列最適化に拡張する。

---

## 6. レイアウト

```
nedo_demo/
├── README.md
├── worlds/
│   ├── bhs_contest1.sdf           Argus BHS シーン（コンベア + センサーアーチ + ULD + 照明 + cinematic cam）
│   └── pickplace_contest3.sdf     Pick & Place セル（PiPER 設置 + 短尺コンベア + ULD + 照明 + cinematic cam）
├── urdf/
│   └── piper_gz.urdf.xacro        PiPER 6DoF + gripper + base pedestal + ros2_control HW IF
├── config/
│   └── piper_controllers.yaml     arm_controller / gripper_controller（いずれも JointTrajectoryController）
├── launch/
│   └── pickplace_contest3.launch.py  gz sim 起動 + xacro 展開 + robot_state_publisher + bridge + controllers spawn
├── scripts/
│   ├── bag_spawner.py             BHS bag 投入 + belt 駆動 + 識別イベント JSONL 出力
│   ├── bhs_annotator.py           Argus HUD ノード（/cinematic → /cinematic_annotated）
│   ├── pickplace_controller.py    PiPER Pick & Place サイクル + bag spawn / attach / detach
│   ├── pickplace_annotator.py     Pick & Place HUD ノード（cycle / phase / ledger）
│   ├── record_video.py            /cinematic_annotated → ffmpeg → fragmented mp4
│   ├── run_bhs_demo.sh            コンテスト1 一発ラン
│   └── run_pickplace_demo.sh      コンテスト3 一発ラン
└── output/
    ├── contest1_bhs_1080p.mp4
    └── contest3_pickplace_1080p.mp4
```

---

## 7. 実行

### 依存

- Ubuntu 24.04 (Noble)
- ROS 2 Jazzy
- Gazebo Sim 8.11（`gz-sim-vendor`）
- `gz_ros2_control`, `ros_gz_bridge`, `ros_gz_image`, `ros_gz_sim`
- `joint_state_broadcaster`, `joint_trajectory_controller`
- `xacro`, `robot_state_publisher`
- Python 3.12, `opencv-python`, `numpy`
- `ffmpeg` 6+
- NVIDIA GPU + ドライバ（センサー Ogre2 EGL レンダリング）

### 動かす

```bash
source /opt/ros/jazzy/setup.bash
export DISPLAY=:0   # アクティブな X セッション or Xvfb

# コンテスト1（BHS 識別ライン）
DUR_S=60 bash scripts/run_bhs_demo.sh
# → output/contest1_bhs_1080p.mp4

# コンテスト3（Pick & Place セル）
CYCLES=4 REC_DUR_S=140 bash scripts/run_pickplace_demo.sh
# → output/contest3_pickplace_1080p.mp4
```

---

## 8. Stage1 開発計画上の役割

| 期間 | マイルストーン | 本リポジトリの位置付け |
|---|---|---|
| 2026/7 | 移動マニピュレータ・識別装置（カメラ）調達と動作確認 | 本リポジトリの URDF / world を実機キャリブレーション値で更新 |
| 2026/8 | AgileX PiPER 統合、Stage1 用識別装置構築開始 | `pickplace_controller.py` から MoveIt 2 経由のプランニングへ昇格 |
| 2026/9 | メイクエリア／ULD シーン拡張 + コンテスト1 シミュ統合 | Argus 推論モデルの合成学習データ生成と本リポジトリの物理リアリズム検証 |
| 2026/10 | Pick & Place 軽量 bag 実証、QUBO 配置計算統合 | QUBO 出力を `pickplace_controller.py` の waypoint シーケンスに直接接続 |
| 2026/11 | 多 ULD 並列、エラーリカバリ、シミュレーション結果まとめ | 群協調制御を本ワールド上で先行検証 |
| 2026/12 | 中間成果物の動画・データセット完成 | 本リポジトリの mp4 / 配置データを成果報告書添付として提出 |
| 2027/1〜2 | 開発成果報告書作成、Stage1 提出 | Stage2 計画書から本リポジトリの拡張ロードマップを引用 |

---

## 9. ライセンス

本リポジトリ内のコード（`scripts/`, `worlds/`, `urdf/`, `launch/`, `config/`）は MIT License。

外部アセット:
- AgileX PiPER メッシュは AgileX Robotics オリジナルライセンス
- 動画 `output/*.mp4` は NEDO Challenge 公募応募の一部として提示
