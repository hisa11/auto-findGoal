#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from collections import deque

class AdvancedYoloGoalDetector(Node):
  def __init__(self):
    super().__init__('advanced_yolo_detector')
    self.bridge = CvBridge()

    # --- 設定 ---
    # 【重要】追従したいゴールの色をここで指定します ('RED', 'BLUE', 'GREEN' のいずれか)
    self.declare_parameter('target_color', 'BLUE')
    self.target_color_name = self.get_parameter(
        'target_color').value.upper()

    # クラスIDのマッピング
    self.color_to_id = {'RED': 0, 'BLUE': 1, 'GREEN': 2}
    self.target_id = self.color_to_id.get(
        self.target_color_name, 1)  # デフォルトはBLUE
    self.rainbow_id = 3

    self.class_names = {0: "RED", 1: "BLUE", 2: "GREEN", 3: "RAINBOW"}
    self.class_colors = {0: (0, 0, 255), 1: (
        255, 0, 0), 2: (0, 255, 0), 3: (255, 255, 255)}

    # モデルの読み込み
    self.model_path = '/home/hisa/ros2_ws/best.onnx'  # ★ONNX形式に変更
    self.get_logger().info(
        f'YOLOモデルを読み込み中... 追従対象: {self.target_color_name}')
    self.model = YOLO(self.model_path, task='detect')
    # ONNXフォーマットの場合、.to('cpu') を呼ぶとエラーになるため削除します。
    # 代わりに、推論時の model() 関数の引数に device='cpu' を渡します。

    # 履歴フィルタの準備
    self.enable_temporal_filter = True
    self.history = deque(maxlen=15)
    self.saturation_scale = 1.8

    # FPS計測用の準備
    self.frame_count = 0
    self.last_time = self.get_clock().now()

    self.latest_depth_image = None

    # 処理を間引く(スキップする)ための変数。例：3フレームに1回しかYOLOを回さない
    self.process_every_n_frames = 3
    self.frame_skip_counter = 0

    # 最新の検出結果（バウンディングボックスとクラス）を保持
    self.last_best_box = None
    self.last_final_class = None

    # サブスクライバーとパブリッシャー
    self.depth_sub = self.create_subscription(
        Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
    self.color_sub = self.create_subscription(
        Image, '/camera/camera/color/image_raw', self.color_callback, 10)
    self.image_pub = self.create_publisher(
        Image, '/yolo/annotated_image', 10)

  def depth_callback(self, msg):
    try:
      self.latest_depth_image = self.bridge.imgmsg_to_cv2(
          msg, desired_encoding='passthrough')
    except Exception as e:
      self.get_logger().error(f'深度画像エラー: {e}')

  def decide_final_class(self, hist):
    """過去の履歴から最終的な色を決定する賢いロジック"""
    valid_hist = [c for c in hist if c is not None]
    if not valid_hist:
      return None

    counts = {0: 0, 1: 0, 2: 0, 3: 0}
    for c in valid_hist:
      if c in counts:
        counts[c] += 1

    total = len(valid_hist)

    # 条件1: RGBが連続で認識される（赤, 青, 緑のうち2種類以上が存在する） -> RAINBOW(3)
    rgb_types_count = (counts[0] > 0) + (counts[1] > 0) + (counts[2] > 0)
    if rgb_types_count >= 2:
      return 3

    # 条件2: RAINBOWの割合が60%以上 -> RAINBOW(3)
    if counts[3] / total >= 0.6:
      return 3

    # 条件3: ターゲット色とRAINBOWが混在している -> ターゲット色を優先維持
    if counts[self.target_id] > 0 and counts[3] > 0:
      return self.target_id

    return max(counts, key=counts.get)

  def color_callback(self, msg):
    if self.latest_depth_image is None:
      return

    try:
      frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except Exception as e:
      return

    # ここからフレームスキップにより負荷を軽くする
    self.frame_skip_counter += 1

    # --- ソフトウェア彩度ブースト ---
    # OpenCVのPython BGR2HSV変換は実はかなり重い処理です。
    # 今回は処理を軽くするため、彩度ブースト処理の解像度を下げて推論するか、
    # もしくは処理をスキップするアプローチを取ります。

    # 毎回YOLOを回すと重いので、指定フレーム数に1回だけ推論する
    if self.frame_skip_counter >= self.process_every_n_frames:
      self.frame_skip_counter = 0

      # --- 推論 ---
      # カメラ元映像(640x480)そのまま、一切リサイズなし・彩度ブーストなしでYOLOに渡す
      # ONNXもimgsz=640で再作成しました
      results = self.model(frame, conf=0.5,
                           imgsz=640, verbose=False, device='cpu')
      boxes = results[0].boxes

      best_box = None
      raw_class = None

      # 検出されたものの中から、指定色(target_id) または 虹色(3) を探す
      if len(boxes) > 0:
        for box in boxes:
          cls_id = int(box.cls[0])
          if cls_id == self.target_id or cls_id == self.rainbow_id:
            best_box = box
            raw_class = cls_id
            break  # ターゲット色か虹ローを見つけたらループを抜ける

      # 今回はリサイズしていないので、スケール戻し処理は不要。そのまま使う
      if best_box is not None:
        best_box_scaled = best_box.xyxy[0].cpu().numpy()
        self.last_best_box = best_box_scaled
      else:
        self.last_best_box = None

      # 履歴の更新
      if raw_class is not None:
        self.history.append(raw_class)
      else:
        self.history.append(None)

      # フィルタリング適用
      final_class = self.decide_final_class(
          self.history) if self.enable_temporal_filter else raw_class

      self.last_final_class = final_class

      # 以降の処理で使うため変数にセット
      best_box_scaled = self.last_best_box

    else:
      # 推論しないフレームはキャシュ（前回の結果）を使い回す
      best_box_scaled = self.last_best_box
      final_class = self.last_final_class

    # 条件に合致するゴールがあり、フィルタ後も対象である場合のみ描画・距離計測
    if best_box_scaled is not None and final_class in [self.target_id, self.rainbow_id]:
      x1, y1, x2, y2 = map(int, best_box_scaled)
      w = x2 - x1
      h = y2 - y1

      # 枠の描画
      color = self.class_colors.get(final_class, (0, 255, 255))
      label = self.class_names.get(final_class, "UNKNOWN")
      cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)

      status_text = f"TRACKING: {label}"
      cv2.putText(frame, status_text, (x1, y1 - 10),
                  cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

      # --- 黒い土台（距離計測ポイント）の計算 ---
      # コの字の下端(y2)から、枠の高さの20%分だけ下を狙う
      offset_pixels = int(h * 0.20)
      target_x = x1 + (w // 2)
      target_y = y2 + offset_pixels

      # 画面外エラーを防ぐ
      target_y = min(target_y, frame.shape[0] - 1)
      target_x = max(0, min(target_x, frame.shape[1] - 1))

      depth_h, depth_w = self.latest_depth_image.shape[:2]
      target_y_depth = min(target_y, depth_h - 1)
      target_x_depth = min(max(0, target_x), depth_w - 1)

      distance_mm = self.latest_depth_image[target_y_depth, target_x_depth]

      # 照準と距離の描画
      cv2.drawMarker(frame, (target_x, target_y), (0, 0, 255),
                     markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
      cv2.putText(frame, f"Dist: {distance_mm} mm", (target_x + 10,
                  target_y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

      # 推論したフレームにだけ表示するメッセージ（ログの垂れ流し防止）
      if self.frame_skip_counter == 0:
        self.get_logger().info(f'[{label}] 追従中... 距離: {distance_mm} mm')

    # FPS（処理速度）の計算と表示
    self.frame_count += 1
    now = self.get_clock().now()
    elapsed_time = (now - self.last_time).nanoseconds / 1e9  # 秒に変換

    # 3秒おきにFPSをログと画像に表示
    if elapsed_time >= 3.0:
      current_fps = self.frame_count / elapsed_time
      self.current_fps_text = f"FPS: {current_fps:.1f}"
      self.get_logger().info(self.current_fps_text)
      self.last_time = now
      self.frame_count = 0

    # 画面右上に常にFPSを描画
    fps_text = getattr(self, "current_fps_text", "FPS: --")
    cv2.putText(frame, fps_text, (frame.shape[1] - 150, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

    # パブリッシュ
    try:
      annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
      self.image_pub.publish(annotated_msg)
    except Exception as e:
      pass

def main(args=None):
  rclpy.init(args=args)
  node = AdvancedYoloGoalDetector()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()
