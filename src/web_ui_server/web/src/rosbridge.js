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
    this._subscribed = new Set();
    this._subscribers = {}; // topic -> [callback, ...]
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
      // 再接続時は済みサブスクリプションを再登録
      for (const topic of this._subscribed) {
        const entries = this._subscribers[topic];
        if (entries && entries.length > 0) {
          this._ws.send(
            JSON.stringify({
              op: "subscribe",
              topic,
              type: entries[0].msgType,
            }),
          );
        }
      }
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

    this._ws.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data);
        if (msg.op === "publish" && msg.topic && this._subscribers[msg.topic]) {
          for (const { callback } of this._subscribers[msg.topic]) {
            callback(msg.msg);
          }
        }
      } catch (e) {
        console.warn("[ROS] onmessage parse error:", e);
      }
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
   * トピックをサブスクライブする
   * @param {string}   topic     トピック名
   * @param {string}   msgType   メッセージ型 (例: 'std_msgs/msg/Bool')
   * @param {function} callback  受信コールバック (msg) => void
   * @returns {function} コールするとサブスクリプションを解除できる関数
   */
  subscribe(topic, msgType, callback) {
    if (!this._subscribers[topic]) {
      this._subscribers[topic] = [];
    }
    const entry = { msgType, callback };
    this._subscribers[topic].push(entry);

    if (!this._subscribed.has(topic)) {
      this._subscribed.add(topic);
      this._send({ op: "subscribe", topic, type: msgType });
    }

    // 返値は登録解除用関数
    return () => {
      const arr = this._subscribers[topic];
      if (!arr) return;
      const idx = arr.indexOf(entry);
      if (idx !== -1) arr.splice(idx, 1);
      if (arr.length === 0) {
        delete this._subscribers[topic];
        this._subscribed.delete(topic);
        this._send({ op: "unsubscribe", topic });
      }
    };
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
// https ページから ws:// は Mixed Content でブロックされるため自動切替
const _WS_PROTO = location.protocol === "https:" ? "wss:" : "ws:";
export const ros = new ROSBridge(`${_WS_PROTO}//${location.hostname}:9090`);
