#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloGoalDetector(Node):
    def __init__(self):
        super().__init__('yolo_goal_detector')
        self.bridge = CvBridge()

        # 【超重要】best.pt の絶対パスに書き換えてください！
        # 例: '/home/hisa/find-goal/best.pt'
        self.model_path = '/home/robotclub/auto-findGoal/best.onnx' 
        
        self.get_logger().info(f'YOLOモデルを読み込み中: {self.model_path}')
        self.model = YOLO(self.model_path)

        self.latest_depth_image = None

        # 深度画像のサブスクライバー
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10)
        
        # カラー画像のサブスクライバー
        self.color_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_callback,
            10)

        # 確認用映像のパブリッシャー（rqt_image_view用）
        self.image_pub = self.create_publisher(Image, '/yolo/annotated_image', 10)

        self.get_logger().info('YOLO Goal Detector ノードが起動しました！')

    def depth_callback(self, msg):
        try:
            # 深度データ(16ビット整数、単位mm)を保存
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'深度画像の変換エラー: {e}')

    def color_callback(self, msg):
        if self.latest_depth_image is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'カラー画像の変換エラー: {e}')
            return

        # YOLOで推論
        results = self.model(frame, verbose=False)

        for result in results:
            boxes = result.boxes
            for box in boxes:
                # 枠の座標を取得
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                w = x2 - x1
                h = y2 - y1

                # 枠を緑色で描画
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, "Goal Frame", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # 黒い土台（ターゲット）の座標を計算
                # コの字の下端(y2)から、枠の高さの20%分だけ下を狙う
                offset_pixels = int(h * 0.20)
                target_x = x1 + (w // 2)
                target_y = y2 + offset_pixels

                # 画面外エラーを防ぐ処理
                target_y = min(target_y, frame.shape[0] - 1)
                target_x = max(0, min(target_x, frame.shape[1] - 1))

                # その座標の距離(mm)を取得
                distance_mm = self.latest_depth_image[target_y, target_x]

                # 狙っている土台に赤い照準を描画
                cv2.drawMarker(frame, (target_x, target_y), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
                
                # 距離を描画
                text = f"Dist: {distance_mm} mm"
                cv2.putText(frame, text, (target_x + 10, target_y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                self.get_logger().info(f'ゴール発見! 距離: {distance_mm} mm')

        # 結果画像をパブリッシュ
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f'画像のパブリッシュエラー: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloGoalDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
