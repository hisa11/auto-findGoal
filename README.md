# auto-findGoal — ROS 2 ワークスペース

## 起動コマンド（毎回）

```bash
# 1. CAN インターフェースを有効化（ロボット起動後に毎回必要）
sudo ip link set can0 up type can bitrate 1000000

# 2. ROS 2 環境を読み込み
source /opt/ros/jazzy/setup.bash
source ~/auto-findGoal/install/setup.bash

# 3. 全サブシステムを一括起動
ros2 launch web_ui_server robot.launch.py
```

ブラウザから `https://<ロボットのIP>:8443` にアクセス。

### 部分起動オプション

```bash
# カメラ・YOLO なし（走行系のみ）
ros2 launch web_ui_server robot.launch.py enable_camera:=false

# HTTP で起動（証明書不要）
ros2 launch web_ui_server robot.launch.py use_https:=false web_port:=8080

# 追従ゴール色を変更（RED / BLUE / GREEN）
ros2 launch web_ui_server robot.launch.py target_color:=RED
```

---

## 別 PC への移行手順

### 1. ROS 2 Jazzy のインストール

```bash
# 公式ドキュメント通りにインストール
# https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
sudo apt update && sudo apt install -y ros-jazzy-desktop
```

### 2. リポジトリをクローン

```bash
cd ~
git clone https://github.com/hisa11/auto-findGoal.git
# best.onnx は git 管理外のため別途コピー（下記参照）
```

### 3. システム依存パッケージを一括インストール

```bash
source /opt/ros/jazzy/setup.bash
sudo rosdep init          # 初回のみ
rosdep update

cd ~/auto-findGoal
sudo rosdep install --from-paths src --ignore-src -r -y
```

#### 追加で必要な apt パッケージ

rosdep では自動インストールされないパッケージを手動で入れます。

```bash
# RealSense ドライバ（カメラを使う場合）
sudo apt install -y ros-jazzy-realsense2-camera ros-jazzy-realsense2-description

# CompressedImage トランスポート（必須・欠けるとカメラ映像が映らない）
sudo apt install -y ros-jazzy-compressed-image-transport

# ROSBridge（WebUI と ROS の WebSocket 通信）
sudo apt install -y ros-jazzy-rosbridge-suite

# SSL 証明書ツール
sudo apt install -y libnss3-tools
```

### 4. Python パッケージのインストール

> **重要**: pyenv が有効な環境では `pip install` が pyenv 側の Python に入ります。
> ROS ノードは `/usr/bin/python3` (system Python) で実行されるため、
> **必ず `/usr/bin/pip3` を使ってシステム側にインストールしてください。**

```bash
# YOLO 推論エンジン（system Python へインストール）
sudo /usr/bin/pip3 install --break-system-packages ultralytics

# ONNX ランタイム（system Python へインストール）
sudo /usr/bin/pip3 install --break-system-packages onnxruntime onnx
```

pyenv が入っている場合、`python3 --version` で確認できるバージョンと
`/usr/bin/python3 --version` が異なる可能性があります。
`cv2` は `ros-jazzy-python3-opencv`（apt 版）が自動でインストールされるため、
`pip install opencv-python` は**しないでください**（バージョン競合で `cv2.putText` 等が壊れます）。

### 5. YOLO モデルファイルのコピー

`best.onnx` は git 管理外のため、旧 PC から手動でコピーしてください。

```bash
# 旧 PC → 新 PC（旧 PC 側で実行）
scp ~/auto-findGoal/best.onnx <新PCのユーザー>@<新PCのIP>:~/auto-findGoal/best.onnx
```

モデルのパスは `src/can_motor_driver/src/advanced_yolo_detector.py` の
`self.model_path` に絶対パスで記載されています。
ユーザー名やディレクトリが変わった場合はここを修正してください。

### 6. SSL 証明書の生成・設定

mkcert で自己署名証明書を生成します。証明書はロボットの IP アドレスに紐付くため、
IP が変わった場合は再生成が必要です。

```bash
# mkcert をインストール
wget -O /usr/local/bin/mkcert https://dl.filippo.io/mkcert/latest?for=linux/amd64
chmod +x /usr/local/bin/mkcert
mkcert -install

# ワークスペースのロボット IP を確認
ip -4 a show | grep inet

# 証明書を発行（IP は実際のものに変更）
cd ~/auto-findGoal/src/web_ui_server/certs
mkcert <ロボットのIP> localhost 127.0.0.1
```

生成されたファイル名を `robot.launch.py` 内の `certfile` / `keyfile` パスに合わせるか、
以下のように launch 引数で上書きします。

```bash
ros2 launch web_ui_server robot.launch.py \
  cert_file:=$HOME/auto-findGoal/src/web_ui_server/certs/<IP>+2.pem \
  key_file:=$HOME/auto-findGoal/src/web_ui_server/certs/<IP>+2-key.pem
```

#### 接続デバイス（スマートフォン等）への CA 証明書インストール

rosbridge の wss:// 接続でエラー (`SSLV3_ALERT_CERTIFICATE_UNKNOWN`) が出る場合、
接続元デバイスが mkcert の Root CA を信頼していません。

```
https://<ロボットのIP>:8443/rootCA.pem
```

上記 URL をデバイスのブラウザで開いてダウンロードし、以下の手順でインストールします。

| デバイス | 手順 |
|---------|------|
| **Android** | ダウンロード後 → 設定 → セキュリティ → CA 証明書 → インストール |
| **iOS/iPadOS** | Safari でダウンロード → 設定 → プロファイル → インストール → 一般 → 証明書信頼設定 → 有効化 |
| **Windows** | ダウンロード後 → 右クリック → 証明書のインストール → ローカルコンピュータ → 信頼されたルート CA |

証明書を入れたくない場合は `use_https:=false web_port:=8080` で HTTP 起動できます
（Gamepad API は HTTPS 必須のため、ゲームパッドは使えなくなります）。

### 7. ビルド

```bash
cd ~/auto-findGoal
source /opt/ros/jazzy/setup.bash
colcon build
```

### 8. 起動確認

```bash
sudo ip link set can0 up type can bitrate 1000000
source ~/auto-findGoal/install/setup.bash
ros2 launch web_ui_server robot.launch.py
```

---

## ハードウェア接続

| デバイス | 接続先 | 備考 |
|---------|-------|------|
| RealSense D455 | **USB 3.0 ポート** | USB 2.1 接続だとフレームレートが 15fps に制限される。`Device USB type: 2.1` の WARN が出たら挿し直す |
| CAN アダプタ | USB | 起動後に `sudo ip link set can0 up type can bitrate 1000000` が必要 |

---

## トラブルシューティング

### カメラ映像が緑画面になる

以下を順番に確認してください。

1. **`compressed-image-transport` が未インストール**  
   `ros topic list` に `/camera/camera/color/image_raw/compressed` が存在しない場合:
   ```bash
   sudo apt install -y ros-jazzy-compressed-image-transport
   ```

2. **前回の realsense プロセスがカメラデバイスを占有している**  
   ```bash
   fuser -k /dev/video*
   pkill -9 realsense2_camera_node
   ```

3. **SSL エラーで rosbridge が切断されている**  
   ブラウザの開発者ツールのコンソールに `WebSocket connection failed` が出ていれば、
   上記「接続デバイスへの CA 証明書インストール」を実施するか、`use_https:=false` で起動。

### `cv2.putText` / `cv2.rectangle` が `img is not a numpy array` で落ちる

`pip install opencv-python` でインストールした cv2 4.11.0 が Python パスに入っている場合に発生。
原因は pyenv 環境で破損した wheel がシステム Python を上書きしているケースが多い。

```bash
# system Python の cv2 バージョンを確認（4.6.x なら正常）
/usr/bin/python3 -c "import cv2; print(cv2.__version__)"

# pyenv 側に opensCV が入っていれば削除
~/.pyenv/versions/*/bin/pip uninstall -y opencv-python opencv-python-headless 2>/dev/null || true
```

### `ModuleNotFoundError: No module named 'onnxruntime'`

system Python に onnxruntime が入っていない場合:

```bash
sudo /usr/bin/pip3 install --break-system-packages onnxruntime onnx
```

### 起動時に `Address already in use`（ポート 8443 / 9090）

前回の launch が正常終了せずポートを占有している。  
`robot.launch.py` に起動前クリーンアップが組み込まれているため、
通常は自動解放されますが、手動でも解放できます:

```bash
fuser -k 8443/tcp
fuser -k 9090/tcp
fuser -k /dev/video*
pkill -9 realsense2_camera_node
```

### `UsbCan: ioctl(SIOCGIFINDEX) failed for 'can0': No such device`

CAN インターフェースが有効化されていません:

```bash
sudo ip link set can0 up type can bitrate 1000000
```

---

## パッケージ構成

```
~/auto-findGoal/
├── best.onnx                    # YOLO モデル（git 管理外・要別途コピー）
├── src/
│   ├── can_motor_driver/        # C++ CAN 制御 + Python YOLO 検出
│   │   └── src/
│   │       └── advanced_yolo_detector.py
│   └── web_ui_server/           # WebUI HTTPS 配信 + 統合 launch
│       ├── certs/               # SSL 証明書（mkcert で生成）
│       ├── launch/
│       │   ├── robot.launch.py  # ← 全サブシステム一括起動
│       │   └── web_ui.launch.py # WebUI サーバー単体起動
│       └── web/
│           ├── index.html       # ポータル（GUNNER / CONTROL 選択）
│           ├── gunner.html      # 砲塔制御・カメラ映像
│           ├── control.html     # 走行・CAN ステータス
│           └── rootCA.pem       # mkcert Root CA（接続デバイスにインストール）
```

---

## トピック一覧

| トピック | 型 | 用途 |
|---------|---|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 走行速度指令（4輪オムニ） |
| `/gunner/yaw_vel` | `std_msgs/Float32` | 砲塔旋回速度指令 |
| `/gunner/fire` | `std_msgs/Bool` | 発射トリガー |
| `/yolo/goal_info` | `geometry_msgs/Point` | YOLO 検出結果（x=中心x, y=中心y, z=幅） |
| `/yolo/target_color` | `std_msgs/String` | 追従色変更コマンド（RED/BLUE/GREEN） |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | カメラ生画像 |
| `/camera/camera/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | WebUI 表示用圧縮画像 |
| `/camera/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 深度画像（距離計測用） |

