/**
 * rosbridge.js
 * ~~~~~~~~~~~~
 * 最小実装の ROSBridge WebSocket クライアント。
 * roslib.js 不要・ローカル閉鎖 LAN 対応。
 *
 * 使い方:
 *   import { ros } from './rosbridge.js';
 *   ros.setStatusEl('my-status-div');                      // 任意
 *   ros.publish('/cmd_vel', 'geometry_msgs/msg/Twist', msg);
 */

export class ROSBridge {
  /**
   * @param {string} url  ws:// または wss:// の WebSocket URL
   */
  constructor(url) {
    this._url = url;
    this._ws = null;
    this._ready = false;
    this._advertised = new Set();
    this._queue = []; // 接続前のメッセージを一時格納
    this._statusEl = null;
    this._connect();
  }

  /** DOM 要素の id を渡すと接続状態をリアルタイム表示する */
  setStatusEl(id) {
    this._statusEl = document.getElementById(id);
    this._applyStatus(this._ready);
  }

  _applyStatus(ok) {
    if (!this._statusEl) return;
    this._statusEl.textContent = ok ? "ROS: 接続済み" : "ROS: 未接続";
    this._statusEl.className = ok ? "ros-status ros-connected" : "ros-status";
  }

  _connect() {
    if (this._statusEl) {
      this._statusEl.textContent = "ROS: 接続中...";
      this._statusEl.className = "ros-status";
    }
    this._ws = new WebSocket(this._url);

    this._ws.onopen = () => {
      this._ready = true;
      this._applyStatus(true);
      console.log("[ROS] 接続完了:", this._url);
      // 接続前にキューされたメッセージをすべて送信
      for (const m of this._queue) this._ws.send(m);
      this._queue = [];
    };

    this._ws.onclose = () => {
      this._ready = false;
      this._advertised.clear();
      this._applyStatus(false);
      console.warn("[ROS] 切断 — 3 秒後に再接続します");
      setTimeout(() => this._connect(), 3000);
    };

    this._ws.onerror = () => {
      this._applyStatus(false);
    };
  }

  _send(obj) {
    const str = JSON.stringify(obj);
    if (this._ready && this._ws.readyState === WebSocket.OPEN) {
      this._ws.send(str);
    } else {
      // キューが溢れないように上限を設定
      if (this._queue.length < 20) this._queue.push(str);
    }
  }

  /**
   * トピックをアドバタイズする（publish 時に自動呼出しされるため手動不要）
   * @param {string} topic      トピック名
   * @param {string} msgType    メッセージ型 (例: 'std_msgs/msg/Float32')
   */
  advertise(topic, msgType) {
    if (this._advertised.has(topic)) return;
    this._advertised.add(topic);
    this._send({ op: "advertise", topic, type: msgType });
  }

  /**
   * メッセージをパブリッシュする
   * @param {string} topic    トピック名
   * @param {string} msgType  メッセージ型
   * @param {object} msg      メッセージ本体 (JSON オブジェクト)
   */
  publish(topic, msgType, msg) {
    this.advertise(topic, msgType);
    this._send({ op: "publish", topic, msg });
  }
}

/** ワークスペース全体で共有するシングルトンインスタンス */
export const ros = new ROSBridge(`ws://${location.hostname}:9090`);
