/**
 * xArm7 control widget — status, progress, joints, pick_place API.
 */
(function (global) {
  'use strict';

  const BLOCKS = [
    { num: 1, desc: '→ A_UP（取货上方）' },
    { num: 2, desc: '夹爪张开' },
    { num: 3, desc: '→ A_DN（下降取货）' },
    { num: 4, desc: '夹爪闭合（抓取）' },
    { num: 6, desc: '→ A_UP（抬起）' },
    { num: 7, desc: '→ MID（过渡）' },
    { num: 8, desc: '→ B_UP（放货上方）' },
    { num: 9, desc: '夹爪张开（放下）' },
    { num: 11, desc: '→ B_DN（结束）' },
  ];

  const JOINT_LIMITS = [
    { min: -360, max: 360 },
    { min: -130, max: 130 },
    { min: -360, max: 360 },
    { min: -360, max: 360 },
    { min: -360, max: 360 },
    { min: -130, max: 130 },
    { min: -360, max: 360 },
  ];

  let pollTimer = null;
  let pollInterval = 1000;
  let lastLogMsg = '';
  let activeCard = null;

  function $(sel, root) {
    return (root || document).querySelector(sel);
  }

  async function fetchTimeout(url, opts = {}, ms = 10000) {
    const ctrl = new AbortController();
    const t = setTimeout(() => ctrl.abort(), ms);
    try {
      const res = await fetch(url, { ...opts, signal: ctrl.signal });
      clearTimeout(t);
      return res;
    } catch (e) {
      clearTimeout(t);
      throw e;
    }
  }

  function appendLog(card, type, msg) {
    const logEl = $( '#armWidgetLog', card);
    if (!logEl) return;
    const time = new Date().toLocaleTimeString('zh-CN');
    const colors = {
      action: '#3B82F6',
      state: '#6B7280',
      success: '#10B981',
      error: '#EF4444',
      warning: '#F59E0B',
    };
    const div = document.createElement('div');
    div.className = 'arm-log-line';
    div.style.color = colors[type] || '#8fa3b8';
    div.innerHTML = `<span class="log-time">${time}</span> <span>${msg}</span>`;
    logEl.prepend(div);
    while (logEl.children.length > 100) logEl.lastChild.remove();
  }

  function renderJointBars(card, jointsDeg) {
    const el = $('#armWidgetJoints', card);
    if (!el || !jointsDeg || jointsDeg.length < 7) return;
    el.innerHTML = jointsDeg.slice(0, 7).map((deg, i) => {
      const lim = JOINT_LIMITS[i] || { min: -360, max: 360 };
      const range = lim.max - lim.min;
      const pct = Math.max(0, Math.min(100, ((deg - lim.min) / range) * 100));
      return `<div class="arm-joint-bar">
        <span class="jl">J${i + 1}</span>
        <span class="jv">${Number(deg).toFixed(1)}°</span>
        <div class="jt"><div class="jf" style="width:${pct}%"></div></div>
      </div>`;
    }).join('');
  }

  function renderProgress(card, data) {
    const bar = $('#armWidgetProgressBar', card);
    const txt = $('#armWidgetBlockText', card);
    const block = data.current_block;
    if (!bar || !txt) return;

    if (block != null && block > 0 && block < 99) {
      const done = BLOCKS.filter((b) => b.num < block).length;
      bar.style.width = Math.round((done / BLOCKS.length) * 100) + '%';
      const cur = BLOCKS.find((b) => b.num === block);
      txt.textContent = `[B${String(block).padStart(2, '0')}] ${cur ? cur.desc : data.state_text || ''}`;
    } else if (block === 99) {
      bar.style.width = '100%';
      txt.textContent = '✅ 循环完成';
    } else {
      bar.style.width = data.is_busy ? '5%' : '0%';
      txt.textContent = data.state_text || (data.online ? '空闲' : '离线');
    }
  }

  function updateUI(card, data) {
    const onlineEl = $('#armWidgetOnline', card);
    const gripEl = $('#armWidgetGripper', card);
    const resultEl = $('#armWidgetLastResult', card);

    const online = data.online === true;
    const state = data.state || (online ? 'idle' : 'offline');
    const stateCn = { idle: '空闲', running: '执行中', error: '错误', offline: '离线' };

    if (onlineEl) {
      onlineEl.innerHTML = online
        ? `<span class="dot on"></span> 在线 · ${stateCn[state] || state}`
        : '<span class="dot off"></span> 离线';
      onlineEl.className = 'arm-online ' + (online ? 'on' : 'off');
    }

    renderProgress(card, data);
    renderJointBars(card, data.joints_deg);

    const gs = data.gripper_state || 'unknown';
    if (gripEl) {
      gripEl.textContent = gs === 'open' ? '🔓 张开' : gs === 'closed' ? '🔒 闭合' : '— 未知';
    }

    if (resultEl && data.last_result) {
      const lr = data.last_result;
      const ok = lr.success ? '✅' : '❌';
      resultEl.textContent = `${ok} ${lr.message || ''}`;
    }

    const busy = data.is_busy || state === 'running';
    card.querySelectorAll('.arm-w-btn').forEach((btn) => {
      if (btn.dataset.always) return;
      btn.disabled = busy || !online;
    });

    if (data.state_history && data.state_history.length) {
      const latest = data.state_history[data.state_history.length - 1];
      if (latest && latest.msg && latest.msg !== lastLogMsg) {
        lastLogMsg = latest.msg;
        appendLog(card, 'state', latest.msg);
      }
    }
  }

  async function pollOnce(card) {
    try {
      const res = await fetchTimeout('/api/arm/status', {}, 3000);
      const data = await res.json();
      updateUI(card, data);

      let next = 1000;
      if (!data.online) next = 3000;
      else if (data.is_busy || data.state === 'running') next = 500;
      else if (data.state === 'error') next = 1000;

      if (next !== pollInterval) {
        pollInterval = next;
        clearInterval(pollTimer);
        pollTimer = setInterval(() => pollOnce(card), pollInterval);
      }
    } catch (e) {
      updateUI(card, { online: false, state: 'offline', joints_deg: [] });
    }
  }

  function startPoll(card) {
    activeCard = card;
    pollInterval = 1000;
    clearInterval(pollTimer);
    pollTimer = setInterval(() => pollOnce(card), pollInterval);
    pollOnce(card);
  }

  function stopPoll() {
    clearInterval(pollTimer);
    pollTimer = null;
    activeCard = null;
  }

  function bindActions(card) {
    $('#armBtnUnlock', card)?.addEventListener('click', async () => {
      const btn = $('#armBtnUnlock', card);
      btn.disabled = true;
      btn.textContent = '解锁中…';
      appendLog(card, 'action', '触发解锁归位');
      try {
        const res = await fetchTimeout('/api/arm/unlock', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' }, 45000);
        const j = await res.json();
        appendLog(card, j.success ? 'success' : 'error', `解锁: ${j.message}`);
        if (global.showToast) global.showToast(j.message, j.success ? 'ok' : 'err');
      } catch (e) {
        appendLog(card, 'error', '解锁超时');
      } finally {
        btn.textContent = '🔓 解锁';
        pollOnce(card);
      }
    });

    $('#armBtnPick', card)?.addEventListener('click', async () => {
      const btn = $('#armBtnPick', card);
      const cycles = parseInt($('#armCyclesSelect', card)?.value || '1', 10);
      btn.disabled = true;
      btn.textContent = '执行中…';
      appendLog(card, 'action', `触发抓取 (cycles=${cycles})`);
      try {
        const res = await fetchTimeout('/api/arm/pick_place', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ cycles }),
        }, 130000);
        const j = await res.json();
        appendLog(card, j.success ? 'success' : 'error', `抓取: ${j.message} (${j.duration_s || 0}s)`);
        if (j.success && j.warning) appendLog(card, 'warning', j.warning);
        if (global.showToast) global.showToast(j.message, j.success ? 'ok' : 'err', 6000);
      } catch (e) {
        appendLog(card, 'error', '抓取请求超时');
      } finally {
        btn.textContent = '🤖 抓取';
        pollOnce(card);
      }
    });

    $('#armBtnGripOpen', card)?.addEventListener('click', async () => {
      appendLog(card, 'action', '夹爪张开');
      try {
        const res = await fetchTimeout('/api/arm/gripper', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ open: true }),
        }, 15000);
        const j = await res.json();
        appendLog(card, j.success ? 'success' : 'error', j.message);
      } catch (e) {
        appendLog(card, 'error', '夹爪超时');
      }
      pollOnce(card);
    });

    $('#armBtnGripClose', card)?.addEventListener('click', async () => {
      appendLog(card, 'action', '夹爪闭合');
      try {
        const res = await fetchTimeout('/api/arm/gripper', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ open: false }),
        }, 15000);
        const j = await res.json();
        appendLog(card, j.success ? 'success' : 'error', j.message);
      } catch (e) {
        appendLog(card, 'error', '夹爪超时');
      }
      pollOnce(card);
    });

    $('#armBtnStop', card)?.addEventListener('click', async () => {
      appendLog(card, 'warning', '发送停止命令');
      try {
        const res = await fetchTimeout('/api/arm/stop', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' }, 3000);
        const j = await res.json();
        appendLog(card, 'warning', j.message || '已发送');
      } catch (e) {
        appendLog(card, 'error', '停止失败');
      }
    });

    $('#armBtnRefresh', card)?.addEventListener('click', () => pollOnce(card));

    $('#armCyclesSelect', card)?.addEventListener('change', async (e) => {
      const cycles = parseInt(e.target.value, 10);
      try {
        await fetchTimeout('/api/arm/cycles', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ cycles }),
        }, 3000);
      } catch (err) {
        /* ignore */
      }
    });
  }

  const ARM_WIDGET_HTML = `
    <div class="arm-widget">
      <div id="armWidgetOnline" class="arm-online">检测中…</div>
      <div class="arm-progress-wrap">
        <div class="arm-progress-track"><div id="armWidgetProgressBar" class="arm-progress-bar"></div></div>
        <div id="armWidgetBlockText" class="arm-block-text">等待开始</div>
      </div>
      <div id="armWidgetJoints" class="arm-joints-compact"></div>
      <div class="arm-grip-row">夹爪: <span id="armWidgetGripper">—</span></div>
      <div id="armWidgetLastResult" class="arm-last-result">—</div>
      <div class="arm-btn-grid">
        <button type="button" class="btn arm-w-btn" id="armBtnUnlock">🔓 解锁</button>
        <button type="button" class="btn arm-w-btn primary" id="armBtnPick">🤖 抓取</button>
        <button type="button" class="btn arm-w-btn arm-btn-stop" id="armBtnStop" data-always="1">🛑 停止</button>
        <button type="button" class="btn arm-w-btn" id="armBtnGripOpen">🔓 开</button>
        <button type="button" class="btn arm-w-btn" id="armBtnGripClose">🔒 合</button>
        <button type="button" class="btn arm-w-btn" id="armBtnRefresh" data-always="1">🔄</button>
      </div>
      <div class="arm-cycles-row">
        <label>次数</label>
        <select id="armCyclesSelect" class="arm-cycles-sel">
          <option value="1">1</option><option value="3">3</option>
          <option value="5">5</option><option value="10">10</option>
        </select>
      </div>
      <div id="armWidgetLog" class="arm-widget-log"></div>
    </div>
  `;

  global.initArmWidget = function initArmWidget(card) {
    const body = card.querySelector('.widget-body');
    if (body) body.innerHTML = ARM_WIDGET_HTML;
    lastLogMsg = '';
    bindActions(card);
    startPoll(card);
  };

  global.destroyArmWidget = function destroyArmWidget() {
    stopPoll();
    lastLogMsg = '';
  };
})(window);
