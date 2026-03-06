# auto-findGoal — ROS 2 ワークスペース

## 起動コマンド（毎回）

```bash
# 1. CAN インターフェースを有効化（ロボット起動後に毎回必要）
sudo ip link set can0 up type can bitrate 1000000

# 2. ROS 2 環境を読み込み
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# 3. 全サブシステムを一括起動
ros2 launch web_ui_server robot.launch.py
```

ブラウザから `https://<ロボットのIP>:8443` にアクセス。

### 部分起動オプション

```bash
# カメラ・YOLO なし（走行系のみ）
ros2 launch web_ui_server robot.launch.py enable_camera:=false

# 走行系なし（カメラ・砲塔のみ）
ros2 launch web_ui_server robot.launch.py enable_drive:=false

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
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
git clone https://github.com/hisa11/auto-findGoal.git src
# best.onnx は git 管理外のため別途コピー（下記参照）
```

### 3. システム依存パッケージを一括インストール

```bash
source /opt/ros/jazzy/setup.bash
sudo rosdep init          # 初回のみ
rosdep update

sudo rosdep install --from-paths src --ignore-src -r -y
```

### 4. Python パッケージのインストール（rosdep 未対応）

```bash
pip install ultralytics
```

### 5. YOLO モデルファイルのコピー

`best.onnx` は git 管理外のため、旧 PC から手動でコピーしてください。

```bash
# 旧 PC → 新 PC（旧 PC 側で実行）
scp ~/ros2_ws/best.onnx <新PCのユーザー>@<新PCのIP>:~/ros2_ws/best.onnx
```

> モデルパスは `src/can_motor_driver/src/advanced_yolo_detector.py` の
> `self.model_path = '/home/hisa/ros2_ws/best.onnx'` に固定されています。
> ユーザー名が変わる場合はここを修正してください。

### 6. SSL 証明書の再生成（IP アドレスが変わる場合）

現在の証明書は `192.168.2.8` 向けに発行されています。
IP が変わる場合は mkcert で再生成してください。

```bash
cd ~/ros2_ws/src/web_ui_server/certs

# mkcert をインストール（未インストールの場合）
sudo apt install libnss3-tools
# バイナリは pixel-landscape-app/ にあります
cp ~/pixel-landscape-app/mkcert-v1.4.4-linux-amd64 /usr/local/bin/mkcert
chmod +x /usr/local/bin/mkcert
mkcert -install

# 新しい IP 向けに証明書を発行（IP は実際のものに変更）
mkcert <新しいIP> localhost 127.0.0.1

# 生成されたファイルをリネームして配置
mv <新しいIP>+*-key.pem <新しいIP>+*.pem .

# server_node.py の cert_file / key_file パラメータを更新するか、
# launch ファイルで上書き指定する
ros2 launch web_ui_server robot.launch.py \
  cert_file:=$PWD/<新しいIP>+*.pem \
  key_file:=$PWD/<新しいIP>+*-key.pem
```

> スマートフォンのブラウザで証明書を信頼するには、
> `http://<IP>:8080` で mkcert の rootCA.pem をダウンロード・インストールするか、
> `use_https:=false` で HTTP 運用してください（Gamepad API は HTTPS 必須）。

### 7. ビルド

```bash
cd ~/ros2_ws
colcon build
```

### 8. 起動確認

```bash
sudo ip link set can0 up type can bitrate 1000000
source install/setup.bash
ros2 launch web_ui_server robot.launch.py
```

---

## パッケージ構成

```
src/
├── can_motor_driver/   # C++ CAN 制御 + Python YOLO 検出
└── web_ui_server/      # WebUI HTTPS 配信 + 統合 launch
    ├── launch/
    │   ├── robot.launch.py      # ← 全サブシステム一括起動
    │   └── web_ui.launch.py     # WebUI サーバー単体起動
    └── web/
        ├── index.html           # ポータル（GUNNER / CONTROL 選択）
        ├── gunner.html          # 砲塔制御・カメラ映像
        └── control.html         # 走行・CAN ステータス
```

## トピック一覧

| トピック          | 型                    | 用途                       |
| ----------------- | --------------------- | -------------------------- |
| `/cmd_vel`        | `geometry_msgs/Twist` | 走行速度指令（4輪オムニ）  |
| `/gunner/yaw_vel` | `std_msgs/Float32`    | 砲塔旋回速度指令           |
| `/gunner/fire`    | `std_msgs/Bool`       | 発射トリガー               |
| `/joy`            | `sensor_msgs/Joy`     | PS4 コントローラー生データ |
