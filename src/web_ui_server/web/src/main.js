import { ros } from "./rosbridge.js";

window.addEventListener("load", () => {
  // 画面をクリックした時にフルスクリーン＆横固定（ブラウザ制限のため）
  document.body.addEventListener(
    "click",
    async () => {
      try {
        if (document.documentElement.requestFullscreen) {
          await document.documentElement.requestFullscreen();
          await screen.orientation.lock("landscape");
        }
      } catch (e) {
        console.error("Orientation lock failed:", e);
      }
    },
    { once: true },
  );
});
let state = {
  focus: 0, // 0: 露出, 1: ゲイン
  exposure: 50,
  gain: 50,
  isAuto: true,
  isLocked: true, // スライダーロック（初期: 施錠）
};

// 切り替え時の誤作動防止用フラグ
let wasBothPressed = false;

// 長押し用のタイマー変数
let repeatTimer = null;
let lastBtn = null;

// ---- ROSBridge 設定 ----
const YAW_TOPIC = "/gunner/yaw_vel"; // std_msgs/msg/Float32
const FIRE_TOPIC = "/gunner/fire"; // std_msgs/msg/Bool
const COLOR_TOPIC = "/yolo/target_color"; // std_msgs/msg/String
const YAW_SCALE = 3.0; // 最大旋回速度 [rad/s]
const STICK_DEAD = 0.08; // スティックのデッドゾーン

// ---- 追従色選択 ----
let currentTargetColor = "BLUE"; // デフォルト

function setTargetColor(color) {
  currentTargetColor = color;
  // ボタンのアクティブ状態を更新
  document.querySelectorAll(".color-btn").forEach((btn) => {
    btn.classList.toggle("color-btn-active", btn.dataset.color === color);
  });
  // ROS トピックに送信
  ros.publish(COLOR_TOPIC, "std_msgs/msg/String", { data: color });
  console.log("[YOLO] target_color →", color);
}

document.querySelectorAll(".color-btn").forEach((btn) => {
  btn.addEventListener("click", (e) => {
    e.stopPropagation();
    setTargetColor(btn.dataset.color);
  });
});

let prevFirePressed = false; // 発射エッジ検出用
let pubFrameCount = 0; // 送信レート制御 (~20 Hz)

function updateModeUI() {
  const container = document.querySelector(".ui-overlay");
  const label = document.getElementById("mode-indicator");
  const frame = document.getElementById("main-frame");

  if (state.isAuto) {
    container.className = "ui-overlay status-auto";
    label.innerText = "AUTO";
    if (frame) {
      frame.classList.remove("status-manual");
      frame.classList.add("status-auto");
    }
  } else {
    container.className = "ui-overlay status-manual";
    label.innerText = "MANUAL";
    if (frame) {
      frame.classList.remove("status-auto");
      frame.classList.add("status-manual");
    }
  }
}

function updateUI() {
  // フォーカス表示の切り替え
  document
    .getElementById("group-exposure")
    .classList.toggle("active", state.focus === 0);
  document
    .getElementById("group-gain")
    .classList.toggle("active", state.focus === 1);

  // バーの高さと数値を更新
  document.getElementById("bar-exposure").style.height = `${state.exposure}%`;
  document.getElementById("val-exposure").innerText = state.exposure;
  document.getElementById("bar-gain").style.height = `${state.gain}%`;
  document.getElementById("val-gain").innerText = state.gain;
}

function updateLockUI() {
  const btn = document.getElementById("btn-lock");
  if (!btn) return;
  if (state.isLocked) {
    btn.textContent = "🔒";
    btn.classList.add("locked");
    btn.classList.remove("unlocked");
  } else {
    btn.textContent = "🔓";
    btn.classList.remove("locked");
    btn.classList.add("unlocked");
  }
}

// ---- YAW インジケーター更新 ----
function updateYawUI(yaw) {
  const valEl = document.getElementById("yaw-val");
  const indEl = document.getElementById("yaw-indicator");
  const mon = document.getElementById("yaw-monitor");
  if (!valEl || !indEl) return;

  const active = !state.isAuto;
  if (mon) mon.classList.toggle("yaw-active", active);

  valEl.textContent = active ? yaw.toFixed(2) : "---";

  // バーを中心から左右に伸ばす
  const norm = Math.max(-1, Math.min(1, yaw / YAW_SCALE));
  const pct = Math.abs(norm) * 50;
  if (norm >= 0) {
    indEl.style.left = "50%";
    indEl.style.right = "auto";
  } else {
    indEl.style.left = "auto";
    indEl.style.right = "50%";
  }
  indEl.style.width = `${pct}%`;
}

function adjustValue(dir) {
  if (state.isLocked) return; // ロック中は変更不可
  const key = state.focus === 0 ? "exposure" : "gain";
  state[key] = Math.max(0, Math.min(100, state[key] + dir));
  updateUI();
}

function loop() {
  const gp = navigator.getGamepads()[0];
  if (!gp) return requestAnimationFrame(loop);

  const l1 = gp.buttons[4].pressed;
  const r1 = gp.buttons[5].pressed;
  const circle = gp.buttons[1].pressed;

  // --- モード切替ロジック ---
  // AUTO→MANUAL: L1+R1同時押しのみ
  // MANUAL→AUTO : L1+R1同時押し または 片方のみ押し
  if (l1 && r1) {
    if (!wasBothPressed) {
      if (state.isAuto) {
        state.isAuto = false; // AUTO→MANUAL
      } else {
        state.isAuto = true; // MANUAL→AUTO
      }
      wasBothPressed = true;
      updateModeUI();
    }
  } else if (!l1 && !r1) {
    wasBothPressed = false;
  } else if ((l1 || r1) && !state.isAuto && !wasBothPressed) {
    // 片方のみ押しでMANUAL→AUTO（同時押しからの指残りは wasBothPressed で防止）
    state.isAuto = true;
    updateModeUI();
  }

  // --- 発射ボタン（常に有効） ---
  const fireBtn = document.getElementById("btn-fire");
  if (fireBtn) {
    if (circle) {
      fireBtn.classList.add("active");
    } else {
      fireBtn.classList.remove("active");
    }
  }

  // --- 発射信号: 押した瞬間のみ送信 ---
  if (circle && !prevFirePressed) {
    ros.publish(FIRE_TOPIC, "std_msgs/msg/Bool", { data: true });
  }
  prevFirePressed = circle;

  // --- MANUAL 時: 右スティック X → 砲塔旋回速度 ---
  const rawYaw = gp.axes[2] ?? 0;
  const yaw = Math.abs(rawYaw) > STICK_DEAD ? rawYaw * YAW_SCALE : 0.0;

  // 20 Hz でパブリッシュ (requestAnimationFrame は ~60 fps のため 3 フレームに 1 回)
  if (++pubFrameCount % 3 === 0) {
    // AUTO → 0 を送り続けて砲塔を停止, MANUAL → 実値を送信
    ros.publish(YAW_TOPIC, "std_msgs/msg/Float32", {
      data: state.isAuto ? 0.0 : yaw,
    });
  }

  updateYawUI(yaw);

  // 十字キー入力判定 (PS4コントローラーのボタンインデックス)
  // 14: Left, 15: Right, 12: Up, 13: Down
  const btns = {
    left: gp.buttons[14].pressed,
    right: gp.buttons[15].pressed,
    up: gp.buttons[12].pressed,
    down: gp.buttons[13].pressed,
  };

  // 左右で選択切り替え（単押し）
  if (btns.left) state.focus = 0;
  if (btns.right) state.focus = 1;

  // 上下で値変更（長押し対応）
  if (btns.up || btns.down) {
    const dir = btns.up ? 1 : -1;
    if (!repeatTimer) {
      adjustValue(dir); // 初回移動
      // 200ms後に高速リピート開始
      repeatTimer = setInterval(() => adjustValue(dir), 50);
    }
  } else {
    clearInterval(repeatTimer);
    repeatTimer = null;
  }

  updateUI();
  requestAnimationFrame(loop);
}

window.addEventListener("gamepadconnected", () => {
  console.log("Gamepad Connected");
  ros.setStatusEl("ros-gunner-status");
  loop();
});

window.addEventListener("gamepaddisconnected", () => {
  // 接続が切れたら砲塔を安全停止
  ros.publish(YAW_TOPIC, "std_msgs/msg/Float32", { data: 0.0 });
  prevFirePressed = false;
  updateYawUI(0);
});

// ロックボタンのタップでロック解除／施錠
document.getElementById("btn-lock")?.addEventListener("click", (e) => {
  e.stopPropagation(); // フルスクリーン起動を防ぐ
  state.isLocked = !state.isLocked;
  updateLockUI();
});

// ============================================================
// RealSense カメラ映像表示
//   /camera/camera/color/image_raw/compressed をサブスクライブし
//   #main-frame の src を base64 JPEG で更新する
// ============================================================
(function setupCamera() {
  const imgEl = document.getElementById("main-frame");
  if (!imgEl) return;

  const CAMERA_TOPIC = "/camera/camera/color/image_raw/compressed";

  ros.subscribe(CAMERA_TOPIC, "sensor_msgs/msg/CompressedImage", (msg) => {
    // rosbridge のバージョンによって msg.data の形式が異なる:
    //   旧バージョン: base64 文字列
    //   新バージョン: 整数配列 (uint8[])
    // 両方に対応する
    let b64;
    if (typeof msg.data === 'string') {
      b64 = msg.data;
    } else {
      // Array<number> または Uint8Array → base64 に変換
      const bytes = msg.data instanceof Uint8Array ? msg.data : new Uint8Array(msg.data);
      let binary = '';
      const chunk = 8192;
      for (let i = 0; i < bytes.length; i += chunk) {
        binary += String.fromCharCode(...bytes.subarray(i, i + chunk));
      }
      b64 = btoa(binary);
    }
    imgEl.src = "data:image/jpeg;base64," + b64;
  });
})();
