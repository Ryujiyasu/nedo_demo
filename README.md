# NEDO Challenge — Gazebo Sim デモワークスペース

NEDO Challenge（空港グランドハンドリング）コンテスト1（手荷物識別）／コンテスト3（積付ロボット）提案書の **§g 類似実績** に添付する動画と再現可能なシミュレーション環境。

Isaac Sim 版（`/data/nedo/sim/`）でも同等のシーンを構築済だが、**ロボット／物体の動きが「キーフレーム＋スクリプテッド表面速度」で人為的に見えた**ため、本ワークスペースで `gz_ros2_control` の `JointTrajectoryController` と `TrackController` プラグインによる **物理駆動版** を別途構築した。Isaac 版の写実的レンダと、本 Gazebo 版の物理リアリズム＋ROS2 統合実装証拠を **相補的に提示** する設計。

提案者：**安河内**（株式会社エムスクエア・ラボ CTO、本コンテスト個人応募代表者）

---

## 1. このリポジトリで示すこと

| 項目 | 提案書での主張 | 本リポジトリで示す根拠 |
|---|---|---|
| ROS2 + Gazebo 物理シミュ実装力 | コンテスト1 §g, コンテスト3 §g | `urdf/`, `worlds/`, `launch/`, `config/` 全部実機ビルド可能で、`scripts/run_*.sh` が一発で起動 |
| AgileX PiPER 6軸協働アーム 実機/シム統合 | コンテスト3 §a, §g | `urdf/piper_gz.urdf.xacro` + `gz_ros2_control` + `JointTrajectoryController` で実物理駆動。実機いちご収穫の Isaac 再現と同型 |
| BHS 25 m/min 動体下マルチモーダル識別 | コンテスト1 §c, §g | `worlds/bhs_contest1.sdf` + `TrackController` で belt 表面速度 0.4167 m/s。`bhs_annotator.py` が Argus 風 HUD で分類・素材・寸法・DB照合スコアをライブ表示 |
| 4分類×7素材×寸法5mm 識別パイプライン | コンテスト1 別紙1 §3.1 | `bag_spawner.py` の `BAG_TYPES`（10 種, 4-class×7-material のサンプル）と `classify()` 関数, `bhs_annotator.py` の検出フィード |
| QUBO ベース積付・順序最適化 | コンテスト3 §a, §f | 本リポジトリでは未実装 (NEDO Q-2 Niobi で実装済の QUBO 基盤を流用する。コンテスト3 開発期間で連携) |

---

## 2. 動画成果物

### 2.1 メイン: コンテスト1 BHS 識別デモ

**`output/contest1_bhs_1080p.mp4`** — 1920×1080 / H.264 / 50 秒 / 7.3 MB

BHS コンベア（4m × 0.6m、25 m/min = 0.4167 m/s）上を、10 種の bag が連続的に流れる絵に、Argus 風スキャナ HUD を重畳：

- **左上ヘッダ**: スキャナ識別子（"ARGUS BHS SCANNER"）、センサ構成（RGBステレオ + 偏光 + Active IR）、BHS 速度
- **右上ステータス**: システム状態、経過時間
- **センサーフレーム下に動的バウンディングボックス**（"SCAN ZONE"）
- **検出コールアウト**（bag が SCAN ZONE 通過時に大きく表示）:
  - 個体 ID、4分類（H/HX/S/SX）、7素材（PLA/MET/FAB/LEA/MIX/VIN/PAP）
  - 包絡直方体寸法（mm 単位）
  - 世界手荷物寸法 DB の照合ブランド（Samsonite Cosmolite, Rimowa, Tumi 等）
  - 照合スコア、推定質量
- **左下ローリングフィード**: 直近5件の検出履歴

実物理エンジン（DART + ODE collision）で belt 表面速度を bag に伝達。bag は剛体物体として摩擦力で押されるため、キャッチアップ・スリップ・停止／加速の挙動が現実的。

### 2.2 おまけ: コンテスト3 Pick & Place スケルトン

**`output/contest3_pickplace_1080p.mp4`** — 1920×1080 / H.264 / 25 秒 / 2.4 MB

AgileX PiPER 6 軸協働アーム + Mobile Mover proxy pedestal + 短尺コンベア + ULD-LD3 風 bin。4 サイクル分の Pick & Place を `joint_trajectory_controller` で実物理駆動：

```
HOME → PREGRASP_OVER_BELT → GRASP_AT_BELT → 
  SPAWN+ATTACH (DetachableJoint at link6) → 
LIFT_FROM_BELT → PREPLACE_OVER_ULD → PLACE_INTO_ULD → 
  DETACH (bag drops into ULD) → 
LIFT_FROM_ULD → HOME → (repeat)
```

`pickplace_annotator.py` が "MOBILE MOVER STACKER" HUD を重畳し、サイクル進捗・現在 bag 種別・Phase・ULD 充填台帳を表示。bag は PiPER のグリッパストローク（4 cm）に合わせて **10 cm キューブ** とした（22-32 kg 級フルサイズは Stage2 のヘビーデューティアームで実装）。

**位置付け**: この動画はあくまで「Mobile Mover + PiPER の ros2_control パイプラインが動くこと」のスケルトン実証。提案書 §g の主要根拠は別途、株式会社エムスクエア・ラボ社内で完成済みの **実機いちご・トマト自律収穫 End-to-End** (申請メンバー Sandesh Athawale 実装、ROS 2 Jazzy + AgileX PiPER + RealSense + YOLO + MoveIt 2) であり、本シムは Stage1 開発開始時点でのシーン設計の出発点として位置づけている。

---

## 3. アーキテクチャ

### 3.1 シミュレーションスタック

```
┌─────────────────────────────────────────────────────────────────┐
│ ROS 2 Jazzy                                                     │
│                                                                 │
│  ┌──────────────────┐  ┌────────────────┐  ┌──────────────┐    │
│  │ pickplace_       │  │ bag_spawner.py │  │ {bhs,pp}_    │    │
│  │ controller.py    │  │ (BHS only)     │  │ annotator.py │    │
│  │ (Contest3 only)  │  │                │  │              │    │
│  │ - arm trajectory │  │ - drives belt  │  │ - OpenCV HUD │    │
│  │ - bag spawn      │  │ - spawns bags  │  │ - JSONL feed │    │
│  │ - detach trigger │  │   + events     │  │ - re-pub img │    │
│  └────────┬─────────┘  └────────┬───────┘  └──────┬───────┘    │
│           │  /arm_controller/   │ gz service       │            │
│           │  joint_trajectory   │ /world/.../create│            │
│           ▼                     ▼                  │            │
│  ┌──────────────────────────────────────────┐     │            │
│  │ controller_manager (gz_ros2_control)     │     │            │
│  │  - JointTrajectoryController × 2         │     │            │
│  │  - JointStateBroadcaster                 │     │            │
│  └────────────────┬─────────────────────────┘     │            │
│                   │                                │            │
│  ┌────────────────▼─────────────────┐  ┌──────────▼─────────┐  │
│  │ gz_ros2_control system plugin    │  │ ros_gz_image       │  │
│  │ (embedded in gz sim process)     │  │ /cinematic →       │  │
│  │  - reads URDF <ros2_control>     │  │ sensor_msgs/Image  │  │
│  │  - exposes hardware interfaces   │  └──────────┬─────────┘  │
│  └────────────────┬─────────────────┘             │            │
└────────────────────┼───────────────────────────────┼────────────┘
                     │ gz transport                   │
                     ▼                                │
┌─────────────────────────────────────────────────────┼────────────┐
│ Gazebo Sim 8.11 (gz-sim)                            │            │
│                                                     │            │
│  Physics (DART + ODE collision) ◀─ TrackController  │            │
│                                  ◀─ DetachableJoint │            │
│                                  ◀─ Sensors (Ogre2) │            │
│                                                     │            │
│  World: bhs_world / pickplace_world                 │            │
│     ├ ground, conveyor (kinematic, surface vel)     │            │
│     ├ piper model (URDF→SDF, 6DoF + gripper)        │            │
│     ├ uld_ld3 (static walls + base)                 │            │
│     ├ bags (dynamic, spawned at runtime)            │            │
│     └ cinematic camera ─────────────────────────────┘            │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
                     │
                     ▼ /cinematic_annotated
              ┌──────────────────┐
              │ record_video.py  │ → ffmpeg → fragmented .mp4
              │ (ROS Image →     │
              │  raw RGB stdin)  │
              └──────────────────┘
```

### 3.2 主要技術判断と理由

| 課題 | 採用解 | 棄却した代替 | 理由 |
|---|---|---|---|
| ベルト上 bag 移動 | `gz-sim-track-controller-system` プラグイン | (a) bag に直接 set_velocity / (b) kinematic pose-sync / (c) PhysxSurfaceVelocity（Isaac） | (a) はサブプロセス overhead で 100ms/call、30Hz update 不可。(b) は物理跳ね返り無し、不自然。Track plugin は belt 表面に ODE friction direction (mu2=150, fdir1=0 1 0) を持たせて bag を物理的に押す。**Isaac の "canned" 感を解消する核** |
| 動画書き出し | `record_video.py` （ROS bridge → ffmpeg pipe → fragmented mp4） | gz CameraVideoRecorder system plugin | gz 内蔵レコーダは強制終了時に moov atom が未確定で再生不可になる事故が頻発。fragmented mp4 (`+frag_keyframe+empty_moov+default_base_moof`) なら途中切れても再生可。さらにフレーム流れる経路を ROS 化することで `bhs_annotator.py` の重畳が同経路で済む |
| グリッパ把持 | `gz-sim-detachable-joint-system`（bag 側に plugin、`child_link=link6`） | (a) 摩擦のみで把持 / (b) 吸着面シミュ | PiPER グリッパ 4 cm ストローク × 10 cm bag では摩擦のみだと滑り出す。DetachableJoint で確実に bag を `link6` 子リンクに剛体接続。URDF→SDF 変換で fixed joint がコラプスされ `gripper_base` が消滅するため、attach 先は `link6` |
| URDF mimic joint | `gripper_right` を fixed joint 化 | URDF `<mimic>` + ros2_control hardware interface | ROS Jazzy の `gz_ros2_control` は mimic 制約付き joint に command_interface が割り当てられると `std::runtime_error("Activated mimic joints cannot have command interfaces")` で異常終了。DetachableJoint で把持を解決するため、右指はビジュアル固定で問題なし |
| 動画 HUD | OpenCV + JSONL イベントストリーム | (a) gz-sim GUI overlay / (b) ffmpeg drawtext | (a) は GUI 必要、ヘッドレス録画不可。(b) はテキストフィルタチェイン地獄になる。Python ROS ノードで `/cinematic` を購読 → OpenCV 描画 → `/cinematic_annotated` を再パブリッシュ → 録画 が拡張容易 |
| Python ROS2 init | `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)` | デフォルト | controller / annotator から `subprocess.run(["gz", ...])` を呼ぶと SIGINT が伝搬し rclpy が即 shutdown → publisher context invalid。No-signals init で完全に切り離す |

### 3.3 アセット選定

| アセット | 採用パス | 注意点 |
|---|---|---|
| AgileX PiPER ビジュアル | `/data/nedo/sim/assets/piper_isaac_sim/piper_description/meshes/*.STL` (binary) | 同名の `agilex_piper_arm_description/meshes/*.STL` は Git LFS ポインタファイルしか無く、Gazebo が ODE assertion で死ぬ |
| Piper xacro マクロ | `urdf/piper_gz.urdf.xacro`（本リポジトリ） | 既存 `agilex_piper_arm_description/urdf/*.xacro` を参考に、`gz_ros2_control` 用 `<ros2_control>` ブロック・絶対パスメッシュ参照・mimic joint 回避を施した独自版 |
| Mobile Mover | `urdf/piper_gz.urdf.xacro` 内の 0.6×0.6×0.5 m silver pedestal proxy | `/data/nedo/sim/assets/mm_urdf/mobile_mover.urdf` は FBX メッシュ参照（Gazebo 非対応）。Stage1 開発期間で DAE 変換予定 |
| ULD コンテナ | `worlds/*.sdf` 内に SDF プリミティブで構築（base + 3 walls） | LD3-AKE 実寸（1.562×1.534×1.626 m）は Contest3 §a で参照、本デモは PiPER リーチ 626 mm に合わせて 0.7×0.7×0.5 m に縮小 |

---

## 4. レイアウト

```
gazebo_sim/
├── README.md                     ← このファイル
├── worlds/
│   ├── bhs_contest1.sdf          BHS コンベア + センサーアーチ + ULD + 照明 + cinematic cam
│   └── pickplace_contest3.sdf    PiPER 設置スペース + 短尺コンベア + ULD + 照明 + cinematic cam
├── urdf/
│   └── piper_gz.urdf.xacro       PiPER 6DoF + gripper + pedestal proxy + ros2_control
├── config/
│   └── piper_controllers.yaml    arm_controller / gripper_controller (JointTrajectoryController)
├── launch/
│   └── pickplace_contest3.launch.py  gz sim + xacro 展開 + robot_state_publisher + bridge + spawner
├── scripts/
│   ├── bag_spawner.py            BHS bag 投入 + belt 駆動 + 検出イベント JSONL 出力
│   ├── bhs_annotator.py          /cinematic 購読 → HUD 重畳 → /cinematic_annotated 配信
│   ├── pickplace_controller.py   PiPER pick-place サイクル + bag spawn/attach/detach
│   ├── pickplace_annotator.py    Pick&Place HUD（cycle/phase/ledger）
│   ├── record_video.py           /cinematic_annotated → ffmpeg → fragmented mp4
│   ├── run_bhs_demo.sh           Contest1 一発ラン (DUR_S=60 で 50s 動画)
│   └── run_pickplace_demo.sh     Contest3 一発ラン (CYCLES=4 REC_DUR_S=140)
└── output/
    ├── contest1_bhs_1080p.mp4
    └── contest3_pickplace_1080p.mp4
```

---

## 5. 再現方法

### 5.1 依存

- Ubuntu 24.04 (Noble) 想定
- ROS 2 Jazzy（`/opt/ros/jazzy/setup.bash`）
- Gazebo Sim 8.11（`gz-sim-vendor`）
- `gz_ros2_control`, `ros_gz_bridge`, `ros_gz_image`, `ros_gz_sim`
- `joint_state_broadcaster`, `joint_trajectory_controller`
- `xacro`, `robot_state_publisher`
- Python 3.12 with `opencv-python`, `numpy`, `python-docx`（提案書編集用）
- `ffmpeg` 6+
- NVIDIA GPU + ドライバ（Ogre2 EGL レンダリング用）

### 5.2 実行

```bash
# Contest 1（BHS）
DUR_S=60 bash scripts/run_bhs_demo.sh
# → output/contest1_bhs_1080p.mp4

# Contest 3（Pick & Place）
CYCLES=4 REC_DUR_S=140 bash scripts/run_pickplace_demo.sh
# → output/contest3_pickplace_1080p.mp4
```

`DISPLAY=:0` がアクティブな X セッション（または Xvfb）に向いている必要あり（センサー（カメラ）レンダリング用）。

---

## 6. 既知の限界と Stage1/2 への持ち越し

| 項目 | 現状（本リポジトリ） | Stage1/2 で解決 |
|---|---|---|
| Mobile Mover 本体表現 | 単純シルバー pedestal | `mobile_mover.urdf` の FBX → DAE 変換、URDF を完全に統合 |
| ULD サイズ | 0.7×0.7×0.5 m（PiPER リーチに合わせて縮小） | LD3-AKE 実寸（1.562×1.534×1.626 m）、ULD 種別パラメタライズ |
| Bag サイズ | 10 cm キューブ（PiPER 把持上限） | 22-32 kg 実寸 bag、ハードシェル/ソフト/異形対応 |
| エンドエフェクタ | 単純 2 指グリッパ + DetachableJoint | **吸着・すくい上げ・取っ手把持の3方式マルチモーダル**（識別装置出力に応じて選択、コンテスト1 と統合運用） |
| QUBO 積付計算 | 未統合 | NEDO Q-2 Niobi で実装済の QUBO 基盤を流用、ULD 内配置・順序決定を計算 |
| 多 ULD 並列 / MM 群協調 | 単機・単 ULD | Stage1 で 2機構成、Stage2 で 4-5機並列 |
| BHS 識別の実機推論連携 | HUD は模擬出力（spawn 時にラベル付け） | Sandesh の YOLO + DINOv2 embedding + DB 検索パイプライン（既に実機いちご収穫で End-to-End 動作）を bag タスクへ転用 |
| Isaac Sim 連携 | 別レポ（`/data/nedo/sim/`） | Domain Randomization 学習データ生成は Isaac 側、ROS2 統合・物理リアリズムは本 Gazebo 側で並列運用 |

---

## 7. 関連リソース

- 申請メンバー実機実証動画（株式会社エムスクエア・ラボ社内）: AgileX PiPER + RealSense + YOLO + MoveIt 2 で果実（イチゴ・トマト）自律検出→把持→配置 End-to-End。Contest 1 識別 + Contest 3 把持の中核技術と完全同型。提案書 §g にて YouTube 限定公開 URL を別途記載。
- NEDO Challenge Q-2 Niobi（QUBO 医療臓器マッチング）: 2026/6 成果報告予定。Contest 3 積付計算と技術構造同型。
- 暗号 OSS hyde（TPM + ML-KEM-768）: crates.io 公開。Mobile Mover 群協調の安全通信層の素材。

---

## 8. ライセンス

本リポジトリ内のコード（`scripts/`, `worlds/`, `urdf/`, `launch/`, `config/`）は MIT License とする。

外部アセット：
- AgileX PiPER メッシュは AgileX Robotics オリジナルライセンスに従う
- 動画 `output/*.mp4` は NEDO Challenge 公募応募の一部として提示するもの

---

提案書本体（`/data/nedo/様式3-1_提案書_コンテスト1_手荷物識別_v2.docx` / `様式3-3_提案書_コンテスト3_積付ロボット_v2.docx`）の §g 「これまでの類似の技術開発・実装実績」項目から本リポジトリの GitHub URL を参照する想定。
