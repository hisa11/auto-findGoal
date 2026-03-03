// 速度順: 上キーで高速方向、下キーで低速方向
const SPEED_ORDER = ["high", "mid", "low"];
let speedIndex = 1; // デフォルト: 中速

// 十字キーの前フレーム状態（エッジ検出用）
let prevUp = false;
let prevDown = false;

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
  updateLoop();
});

window.addEventListener("gamepaddisconnected", () => {
  document.getElementById("gamepad-status").innerText =
    "Controller: Disconnected";
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
      // 発射信号送信処理をここに追加
    } else {
      fireBtn.classList.remove("active");
    }
  }

  // --- 十字キー上下で速度切替（エッジ検出: 押した瞬間のみ）---
  // 12: Up, 13: Down
  const up = gp.buttons[12].pressed;
  const down = gp.buttons[13].pressed;

  if (up && !prevUp) setSpeed(speedIndex - 1); // 上 → 高速方向
  if (down && !prevDown) setSpeed(speedIndex + 1); // 下 → 低速方向

  prevUp = up;
  prevDown = down;

  requestAnimationFrame(updateLoop);
}

// CANリセットボタン
document.getElementById("btn-can-reset")?.addEventListener("click", () => {
  // TODO: 実際のCANリセット信号を送信する処理をここに追加
  console.log("CAN RESET sent");
});
