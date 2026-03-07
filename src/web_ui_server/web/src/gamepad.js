import { ros } from "./rosbridge.js";

// 速度順: 上キーで高速方向、下キーで低速方向
const SPEED_ORDER = ["high", "mid", "low"];
let speedIndex = 1; // デフォルト: 中速

// ---- ROSBridge 設定 ----
const CMD_TOPIC = "/cmd_vel";
const CMD_TYPE = "geometry_msgs/msg/Twist";
const MAX_LINEAR = 1.0; // 最大直動速度 [m/s]
const MAX_ANGULAR = 2.0; // 最大旋回速度 [rad/s]
const STICK_DEAD = 0.08; // デッドゾーン

// ---- C610 モータートピック ----
const C610_RPM_TOPIC = "/c610/target_rpm";
const C610_RPM_TYPE = "std_msgs/msg/Int32";
const C610_RPM_HI = 2000; // △ ボタン時の目標 RPM
const C610_RPM_STOP = 0; // × ボタン時の目標 RPM (停止)

// 速度倍率マップ
const SPEED_MULT = { high: 1.0, mid: 0.6, low: 0.3 };

let pubFrameCount = 0; // ~20 Hz 送信用

function applyDead(v) {
  return Math.abs(v) > STICK_DEAD ? v : 0.0;
}

// ---- CAN ステータス UI 更新 ----
function updateCanStatus(canId, isOk) {
  const container = document.getElementById(`${canId}-status`);
  const text = document.getElementById(`${canId}-text`);
  if (!container || !text) return;
  if (isOk) {
    container.className = "can-item normal";
    text.textContent = "通信正常";
  } else {
    container.className = "can-item error";
    text.textContent = "通信エラー";
  }
}

// CAN0 / CAN1 ステータスをリアルタイムでサブスクライブ
ros.subscribe("/can_status/can0", "std_msgs/msg/Bool", (msg) => {
  updateCanStatus("can0", msg.data);
});
ros.subscribe("/can_status/can1", "std_msgs/msg/Bool", (msg) => {
  updateCanStatus("can1", msg.data);
});

/** オムニ方向表示 — の dot と数値を更新 */
function updateOmniDisplay(vx, vy, wz) {
  const dot = document.getElementById("omni-dot");
  if (dot) {
    // pad は 80×80 px, 有効半径 = 30 px
    const r = 30;
    const dx = (vy / MAX_LINEAR) * r; // Y 方向 → 画面 X
    const dy = -(vx / MAX_LINEAR) * r; // X 方向 → 画面 Y (上が前進)
    dot.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;
    // 移動中はは緑、停止中は暗く
    const moving = Math.abs(vx) > 0.01 || Math.abs(vy) > 0.01;
    dot.style.background = moving ? "#00ff00" : "#1a4a1a";
    dot.style.boxShadow = moving ? "0 0 10px #00ff00" : "none";
  }
  const el = (id) => document.getElementById(id);
  if (el("omni-vx")) el("omni-vx").textContent = vx.toFixed(2);
  if (el("omni-vy")) el("omni-vy").textContent = vy.toFixed(2);
  if (el("omni-wz")) el("omni-wz").textContent = wz.toFixed(2);
  // 旋回インジケーター
  const wzBar = document.getElementById("wz-bar");
  if (wzBar) {
    const pct = (Math.abs(wz) / MAX_ANGULAR) * 50;
    if (wz >= 0) {
      wzBar.style.left = "50%";
      wzBar.style.right = "auto";
    } else {
      wzBar.style.left = "auto";
      wzBar.style.right = "50%";
    }
    wzBar.style.width = `${pct}%`;
  }
}

// 十字キーの前フレーム状態（エッジ検出用）
let prevUp = false;
let prevDown = false;

// PS4 フェイスボタンの前フレーム状態
let prevTriangle = false; // button[3]
let prevCross = false; // button[0]

function setSpeed(index) {
  speedIndex = Math.max(0, Math.min(SPEED_ORDER.length - 1, index));
  const value = SPEED_ORDER[speedIndex];
  const radio = document.querySelector(`input[name="speed"][value="${value}"]`);
  if (radio) radio.checked = true;
}

// PS4コントローラーなどの接続を検知
window.addEventListener("gamepadconnected", (e) => {
  console.log("コントローラー接続:", e.gamepad.id);
  document.getElementById("gamepad-status").innerText =
    "PS4 Controller: Connected";
  ros.setStatusEl("ros-ctrl-status");
  updateLoop();
});

window.addEventListener("gamepaddisconnected", () => {
  document.getElementById("gamepad-status").innerText =
    "Controller: Disconnected";
  // 安全停止: コントローラゼロを送信
  ros.publish(CMD_TOPIC, CMD_TYPE, {
    linear: { x: 0.0, y: 0.0, z: 0.0 },
    angular: { x: 0.0, y: 0.0, z: 0.0 },
  });
  updateOmniDisplay(0, 0, 0);
});

function updateLoop() {
  const gamepads = navigator.getGamepads();
  if (!gamepads[0]) return;

  const gp = gamepads[0];

  // --- Circleボタン (button[1]) で発射ボタンをアクティブに ---
  const circle = gp.buttons[1].pressed;
  const fireBtn = document.getElementById("btn-fire-ctrl");
  if (fireBtn) {
    if (circle) {
      fireBtn.classList.add("active");
    } else {
      fireBtn.classList.remove("active");
    }
  }

  // --- 4輪オムニ制御: 左スティック XY + 右スティック X ---
  // axes[0]: 左X(左径=-1), axes[1]: 左Y(上=-1), axes[2]: 右X(左径=-1)
  const mult = SPEED_MULT[SPEED_ORDER[speedIndex]] ?? 0.6;
  const vx = -applyDead(gp.axes[1] ?? 0) * MAX_LINEAR * mult; // 前進/後退
  const vy = -applyDead(gp.axes[0] ?? 0) * MAX_LINEAR * mult; // 左右ストレーフ
  const wz = -applyDead(gp.axes[2] ?? 0) * MAX_ANGULAR * mult; // 旋回

  // ~20 Hz で /cmd_vel にパブリッシュ
  if (++pubFrameCount % 3 === 0) {
    ros.publish(CMD_TOPIC, CMD_TYPE, {
      linear: { x: vx, y: vy, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: wz },
    });
  }

  updateOmniDisplay(vx, vy, wz);

  // --- 十字キー上下で速度切替（エッジ検出: 押した瞬間のみ）---
  // 12: Up, 13: Down
  const up = gp.buttons[12].pressed;
  const down = gp.buttons[13].pressed;

  if (up && !prevUp) setSpeed(speedIndex - 1); // 上 → 高速方向
  if (down && !prevDown) setSpeed(speedIndex + 1); // 下 → 低速方向

  prevUp = up;
  prevDown = down;

  // --- △ (button[3]): C610 全モーター 8000 RPM ---
  const triangle = gp.buttons[3].pressed;
  if (triangle && !prevTriangle) {
    ros.publish(C610_RPM_TOPIC, C610_RPM_TYPE, { data: C610_RPM_HI });
    console.log("[C610] △ pressed → target_rpm =", C610_RPM_HI);
  }
  prevTriangle = triangle;

  // --- × (button[0]): C610 全モーター停止 ---
  const cross = gp.buttons[0].pressed;
  if (cross && !prevCross) {
    ros.publish(C610_RPM_TOPIC, C610_RPM_TYPE, { data: C610_RPM_STOP });
    console.log("[C610] × pressed → target_rpm =", C610_RPM_STOP);
  }
  prevCross = cross;

  requestAnimationFrame(updateLoop);
}

// CANリセットボタン
document.getElementById("btn-can-reset")?.addEventListener("click", () => {
  // TODO: 実際のCANリセット信号を送信する処理をここに追加
  console.log("CAN RESET sent");
});
