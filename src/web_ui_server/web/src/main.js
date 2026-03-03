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
      // console.log("FIRE!"); // ここに発射信号送信
    } else {
      fireBtn.classList.remove("active");
    }
  }

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
  loop();
});

// ロックボタンのタップでロック解除／施錠
document.getElementById("btn-lock")?.addEventListener("click", (e) => {
  e.stopPropagation(); // フルスクリーン起動を防ぐ
  state.isLocked = !state.isLocked;
  updateLockUI();
});
