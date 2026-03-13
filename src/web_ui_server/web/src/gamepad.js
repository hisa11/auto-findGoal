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

// ---- C610 モータートピック (モーターごとに異なるトピック) ----
const C610_RPM_TYPE = "std_msgs/msg/Int32";
const C610_TOPICS = {
  1: "/c610/motor1/target_rpm",
  2: "/c610/motor2/target_rpm",
  3: "/c610/motor3/target_rpm",
  4: "/c610/motor4/target_rpm",
  5: "/c610/motor5/target_rpm",
  6: "/c610/motor6/target_rpm",
  7: "/c610/motor7/target_rpm"
};

// モーター別の目標RPM管理
const motorTargets = {
  1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 7: 0
};
// 前回パブリッシュした値（変化時のみ送信するため）
const motorLastSent = {
  1: null, 2: null, 3: null, 4: null, 5: null, 6: null, 7: null
};

// 速度倍率マップ
const SPEED_MULT = { high: 1.0, mid: 0.6, low: 0.3 };

let pubFrameCount = 0; // ~20 Hz 送信用

// PS4ボタン前フレーム状態
let prevCircle = false; // button[1]
let prevSquare = false; // button[2]
let prevTriangle = false; // button[3]
let prevCross = false; // button[0]
let prevDpadRight = false; // button[15]
let prevDpadLeft = false;  // button[14]
let prevDpadUp = false;   // button[12]
let prevDpadDown = false; // button[13]

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

// モーター更新関数: 指定したモーターのRPMを送信（値が変化した時のみ）
function publishMotorRPM(motorId, rpm) {
  motorTargets[motorId] = rpm;
  if (motorLastSent[motorId] === rpm) return; // 変化なし → スキップ
  motorLastSent[motorId] = rpm;
  const topic = C610_TOPICS[motorId];
  if (topic) {
    ros.publish(topic, C610_RPM_TYPE, { data: rpm });
    console.log(`[Motor ${motorId}] RPM = ${rpm}`);
  }
}

// ID1,2,3,4から目標値が0以外のものがあるか判定
function hasMotorTarget() {
  return [1, 2, 3, 4].some(id => motorTargets[id] !== 0);
}

// ID7を制御: ID1-4がいずれか0でなければ-1000、全て0なら0
function updateMotor7() {
  const target7 = hasMotorTarget() ? 4000 : 0;
  publishMotorRPM(7, target7);
}

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
  // 全モーター停止
  for (let i = 1; i <= 7; i++) {
    motorTargets[i] = 0;
  }
});

function updateLoop() {
  const gamepads = navigator.getGamepads();
  if (!gamepads[0]) return;

  const gp = gamepads[0];

  // --- PSボタン (button[16]) で発射ボタンをアクティブに ---
  const psButton = gp.buttons[16]?.pressed || false;
  const fireBtn = document.getElementById("btn-fire-ctrl");
  if (fireBtn) {
    if (psButton) {
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

  // ---- C610 モーター制御 ----
  
  // --- Circleボタン (button[1]): ID1,2 を -10000rpm ---
  const circle = gp.buttons[1]?.pressed || false;
  if (circle && !prevCircle) {
    publishMotorRPM(1, -10000);
    publishMotorRPM(2, -10000);
  }
  prevCircle = circle;

  // --- Squareボタン (button[2]): ID1,2 を 0rpm ---
  const square = gp.buttons[2]?.pressed || false;
  if (square && !prevSquare) {
    publishMotorRPM(1, 0);
    publishMotorRPM(2, 0);
  }
  prevSquare = square;

  // --- 十字キー右 (button[15]): ID3 -10000, ID4 10000 ---
  const dpadRight = gp.buttons[15]?.pressed || false;
  if (dpadRight && !prevDpadRight) {
    publishMotorRPM(3, -10000);
    publishMotorRPM(4, 10000);
  }
  prevDpadRight = dpadRight;

  // --- 十字キー左 (button[14]): ID3,4 を 0rpm ---
  const dpadLeft = gp.buttons[14]?.pressed || false;
  if (dpadLeft && !prevDpadLeft) {
    publishMotorRPM(3, 0);
    publishMotorRPM(4, 0);
    console.log("[DPad Left] pressed → ID3,4 = 0");
  }
  prevDpadLeft = dpadLeft;

  // --- 三角ボタン (button[3]): ID5 2000rpm ---
  const triangle = gp.buttons[3]?.pressed || false;
  if (triangle && !prevTriangle) {
    publishMotorRPM(5, 3000);
    console.log("[Triangle] pressed → ID5 = 2000");
  }

  // --- バツボタン (button[0]): ID5 -2000rpm ---
  const cross = gp.buttons[0]?.pressed || false;
  if (cross && !prevCross) {
    publishMotorRPM(5, -2000);
    console.log("[Cross] pressed → ID5 = -2000");
  }

  // --- 三角またはバツボタンを離すと ID5 = 0rpm ---
  if (!triangle && !cross && (prevTriangle || prevCross)) {
    publishMotorRPM(5, 0);
    console.log("[Triangle/Cross] released → ID5 = 0");
  }

  prevTriangle = triangle;
  prevCross = cross;

  // --- 十字キー上 (button[12]): ID6 3000rpm ---
  const dpadUp = gp.buttons[12]?.pressed || false;
  if (dpadUp && !prevDpadUp) {
    publishMotorRPM(6, 3000);
    console.log("[DPad Up] pressed → ID6 = 3000");
  }

  // --- 十字キー下 (button[13]): ID6 -3000rpm ---
  const dpadDown = gp.buttons[13]?.pressed || false;
  if (dpadDown && !prevDpadDown) {
    publishMotorRPM(6, -3000);
    console.log("[DPad Down] pressed → ID6 = -3000");
  }

  // --- 十字キー上下どちらも離されたとき: ID6 0rpm ---
  if (!dpadUp && !dpadDown && (prevDpadUp || prevDpadDown)) {
    publishMotorRPM(6, 0);
    console.log("[DPad Up/Down] released → ID6 = 0");
  }

  // prev 更新は判定の後
  prevDpadUp = dpadUp;
  prevDpadDown = dpadDown;

  // --- ID1,2,3,4から目標値が0以外のものがあるか判定してID7を制御 ---
  updateMotor7();

  requestAnimationFrame(updateLoop);
}

// CANリセットボタン
document.getElementById("btn-can-reset")?.addEventListener("click", () => {
  // TODO: 実際のCANリセット信号を送信する処理をここに追加
  console.log("CAN RESET sent");
});
