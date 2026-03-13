"""
web_ui_server.server_node
~~~~~~~~~~~~~~~~~~~~~~~~~
LAN内制御用WebアプリをHTTPS配信するROS2ノード。

起動方法:
    ros2 run web_ui_server server_node
    または
    ros2 launch web_ui_server web_ui.launch.py
"""

import os
import socket
import ssl
import threading
from http.server import SimpleHTTPRequestHandler, HTTPServer
from functools import partial

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class _MimeHandler(SimpleHTTPRequestHandler):
  """MIME類推を強化したファイルサーバーハンドラ。"""

  # ES Modules を正しく扱うために application/javascript を明示
  extensions_map = {
      **SimpleHTTPRequestHandler.extensions_map,  # type: ignore[arg-type]
      '.js': 'application/javascript',
      '.mjs': 'application/javascript',
      '.css': 'text/css',
      '.json': 'application/json',
      '.svg': 'image/svg+xml',
      '.webp': 'image/webp',
      '.ico': 'image/x-icon',
      '.pem': 'text/plain',        # ブラウザからアクセスされないが念のため
  }

  def log_message(self, fmt: str, *args):  # noqa: D401
    """ROS2ロガー経由でアクセスログを出力する。"""
    if self._ros_logger is not None:
      self._ros_logger.info(f"[HTTP] {self.address_string()} - {fmt % args}")

  # クラス変数としてロガーをセット（スレッドセーフに参照できるよう）
  _ros_logger = None


def _make_handler(directory: str, logger):
  """サーブするディレクトリとロガーを固定したハンドラクラスを生成する。"""
  _MimeHandler._ros_logger = logger

  class BoundHandler(_MimeHandler):
    def __init__(self, *args, **kwargs):
      super().__init__(*args, directory=directory, **kwargs)

  return BoundHandler


class _ReusePortHTTPServer(HTTPServer):
  """SO_REUSEADDR + SO_REUSEPORT を有効にしたHTTPServer。

  前回の起動プロセスが残留していてもポートを強制的に再利用できる。
  """
  allow_reuse_address = True

  def server_bind(self):
    try:
      self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except (AttributeError, OSError):
      pass  # SO_REUSEPORT 非対応カーネルでも続行
    super().server_bind()


class WebUIServerNode(Node):
  """pixel-landscape-appのWebファイルをHTTPSで配信するノード。"""

  def __init__(self):
    super().__init__('web_ui_server')

    # ---- ROS2パラメータ宣言 ----
    pkg_share = get_package_share_directory('web_ui_server')

    self.declare_parameter('host', '0.0.0.0')
    self.declare_parameter('port', 443)
    self.declare_parameter('web_dir', os.path.join(pkg_share, 'web'))
    self.declare_parameter('cert_file', os.path.join(
        pkg_share, 'certs', '192.168.2.200+2.pem'))
    self.declare_parameter('key_file', os.path.join(
        pkg_share, 'certs', '192.168.2.200+2-key.pem'))
    self.declare_parameter('use_https', True)

    host = self.get_parameter('host').value
    port = self.get_parameter('port').value
    web_dir = self.get_parameter('web_dir').value
    cert_file = self.get_parameter('cert_file').value
    key_file = self.get_parameter('key_file').value
    use_https = self.get_parameter('use_https').value

    if not os.path.isdir(web_dir):
      self.get_logger().error(f'web_dir が存在しません: {web_dir}')
      return

    handler = _make_handler(web_dir, self.get_logger())

    # ---- HTTPサーバー作成 ----
    # portが443の場合は root権限が必要なため、必要に応じて8443に変更してください
    self._server = _ReusePortHTTPServer((host, port), handler)

    if use_https:
      if not os.path.isfile(cert_file) or not os.path.isfile(key_file):
        self.get_logger().warn(
            f'証明書ファイルが見つかりません。HTTP（非TLS）で起動します。\n'
            f'  cert: {cert_file}\n'
            f'  key:  {key_file}'
        )
      else:
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(certfile=cert_file, keyfile=key_file)
        self._server.socket = ctx.wrap_socket(
            self._server.socket, server_side=True
        )
        self.get_logger().info('TLSを有効化しました。')

    scheme = 'https' if use_https else 'http'
    self.get_logger().info(
        f'WebUIサーバー起動: {scheme}://{host}:{port}  '
        f'(web_dir={web_dir})'
    )

    # ---- バックグラウンドスレッドで起動 ----
    self._thread = threading.Thread(
        target=self._server.serve_forever, daemon=True
    )
    self._thread.start()

  def destroy_node(self):
    if hasattr(self, '_server'):
      self._server.shutdown()
    super().destroy_node()


def main(args=None):
  rclpy.init(args=args)
  node = WebUIServerNode()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
  main()
