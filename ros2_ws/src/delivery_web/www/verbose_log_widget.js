/**
 * Verbose log widget — per-module toggles + log viewer.
 */
(function (global) {
  'use strict';

  const VERBOSE_MODULES = [
    { id: 'agv', name: '🚗 AGV 状态' },
    { id: 'laser', name: '☁️ 激光点云' },
    { id: 'map', name: '🗺️ 地图' },
    { id: 'station', name: '📍 站点' },
    { id: 'arm', name: '🤖 机械臂' },
    { id: 'camera', name: '📷 相机' },
    { id: 'nav', name: '🧭 导航' },
    { id: 'network', name: '🌐 网络' },
    { id: 'system', name: '💻 系统' },
  ];

  const MODULE_LABELS = Object.fromEntries(VERBOSE_MODULES.map((m) => [m.id, m.name]));

  function fetchJson(url, opts, timeoutMs) {
    const ms = timeoutMs || 8000;
    const ctrl = new AbortController();
    const t = setTimeout(() => ctrl.abort(), ms);
    return fetch(url, { ...opts, signal: ctrl.signal })
      .then((r) => r.json())
      .finally(() => clearTimeout(t));
  }

  let _lastVerboseStatusKey = '';

  function statusKey(status) {
    return Object.entries(status || {})
      .sort(([a], [b]) => a.localeCompare(b))
      .map(([k, v]) => `${k}:${v ? 1 : 0}`)
      .join('|');
  }

  function notifyVerboseStatusChange(status, force) {
    const key = statusKey(status);
    if (!force && key === _lastVerboseStatusKey) return;
    _lastVerboseStatusKey = key;
    if (typeof global.onVerboseStatusChange === 'function') {
      global.onVerboseStatusChange(status);
    }
  }

  function renderVerboseModules(status) {
    const container = document.getElementById('verboseLogModules');
    if (!container) return;

    container.innerHTML = VERBOSE_MODULES.map((m) => {
      const enabled = !!status[m.id];
      const dotColor = enabled ? '#10B981' : '#9CA3AF';
      const dotText = enabled ? '🟢 开' : '🔴 关';
      const toggleColor = enabled ? '#E5364A' : '#E5E7EB';
      return `
        <div class="verbose-module-row">
          <span style="font-size:12px;color:#2D2D2D;">${m.name}</span>
          <div style="display:flex;align-items:center;gap:8px;">
            <span style="font-size:11px;color:${dotColor};">${dotText}</span>
            <button type="button" class="verbose-toggle-btn" data-module="${m.id}" data-enable="${!enabled}"
              style="background:${enabled ? '#E5364A' : '#fff'};
                     color:${enabled ? '#fff' : '#6B7280'};
                     border:1px solid ${toggleColor};
                     border-radius:12px;padding:2px 12px;
                     cursor:pointer;font-size:11px;font-weight:600;">
              ${enabled ? 'ON' : 'OFF'}
            </button>
          </div>
        </div>`;
    }).join('');

    container.querySelectorAll('.verbose-toggle-btn').forEach((btn) => {
      btn.addEventListener('click', () => {
        toggleVerboseModule(btn.dataset.module, btn.dataset.enable === 'true');
      });
    });
  }

  async function fetchVerboseLogStatus() {
    try {
      const status = await fetchJson('/api/log/status');
      renderVerboseModules(status);
      return status;
    } catch (e) {
      console.error('Failed to fetch log status:', e);
      return null;
    }
  }

  async function toggleVerboseModule(module, enable) {
    try {
      const data = await fetchJson('/api/log/toggle', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ module, enable }),
      });
      if (data.success) {
        await fetchVerboseLogStatus();
        refreshVerboseLogs();
        notifyVerboseStatusChange(await fetchJson('/api/log/status'), true);
      }
    } catch (e) {
      console.error('Toggle failed:', e);
    }
  }

  async function toggleAllVerbose(enable) {
    try {
      const data = await fetchJson('/api/log/toggle_all', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ enable }),
      });
      if (data.success) {
        await fetchVerboseLogStatus();
        notifyVerboseStatusChange(await fetchJson('/api/log/status'), true);
      }
    } catch (e) {
      console.error('Toggle all failed:', e);
    }
  }

  async function refreshVerboseLogs() {
    const module = document.getElementById('verboseLogModule')?.value || 'arm';
    const lines = document.getElementById('verboseLogLines')?.value || 100;
    const viewer = document.getElementById('verboseLogViewer');
    if (!viewer) return;

    viewer.textContent = '加载中...';

    try {
      const data = await fetchJson(`/api/log/recent?module=${encodeURIComponent(module)}&lines=${lines}`);
      if (data.lines && data.lines.length > 0) {
        viewer.innerHTML = data.lines
          .map((line) => {
            let color = '#6B7280';
            if (line.includes('[ERROR]')) color = '#EF4444';
            else if (line.includes('[WARNING]')) color = '#F59E0B';
            else if (line.includes('[DEBUG]')) color = '#8B5CF6';
            else if (line.includes('[INFO]')) color = '#2D2D2D';
            const esc = line.trim().replace(/&/g, '&amp;').replace(/</g, '&lt;');
            return `<div style="color:${color};">${esc}</div>`;
          })
          .join('');
      } else {
        viewer.innerHTML = '<span style="color:#666;">无日志记录（该模块可能未开启全量日志）</span>';
      }
    } catch (e) {
      viewer.innerHTML = `<span style="color:#f44336;">加载失败: ${e.message}</span>`;
    }
  }

  function exportVerboseLogs() {
    const viewer = document.getElementById('verboseLogViewer');
    if (!viewer) return;
    const text = viewer.innerText || '';
    const blob = new Blob([text], { type: 'text/plain;charset=utf-8' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = `verbose_log_${Date.now()}.txt`;
    a.click();
    URL.revokeObjectURL(a.href);
  }

  function renderVerboseLogCard(card) {
    const panel = card.querySelector('.widget-body');
    const moduleOptions = VERBOSE_MODULES.map(
      (m) => `<option value="${m.id}"${m.id === 'arm' ? ' selected' : ''}>${m.name.replace(/^[^\s]+\s/, '')}</option>`
    ).join('');

    panel.innerHTML = `
      <div id="verboseLogModules" style="margin-bottom:12px;"></div>
      <div style="display:flex;gap:6px;margin-bottom:10px;">
        <button type="button" id="btnVerboseAllOn" class="btn btn-sm" style="flex:1;color:#059669;border-color:#10B981">全部开启</button>
        <button type="button" id="btnVerboseAllOff" class="btn btn-sm" style="flex:1;color:#DC2626;border-color:#EF4444">全部关闭</button>
      </div>
      <div style="border-top:1px solid rgba(0,0,0,0.06);padding-top:8px;">
        <div style="display:flex;gap:6px;align-items:center;margin-bottom:6px;flex-wrap:wrap;">
          <span style="font-size:12px;color:#6B7280;">模块:</span>
          <select id="verboseLogModule" class="arm-cycles-sel">${moduleOptions}</select>
          <span style="font-size:12px;color:#6B7280;">行数:</span>
          <select id="verboseLogLines" class="arm-cycles-sel">
            <option value="50">50</option>
            <option value="100" selected>100</option>
            <option value="500">500</option>
          </select>
          <button type="button" id="btnVerboseRefresh" class="btn btn-sm">刷新</button>
          <button type="button" id="btnVerboseExport" class="btn btn-sm">导出</button>
        </div>
        <div id="verboseLogViewer" class="log-panel" style="max-height:200px;font-size:11px;color:#6B7280;white-space:pre-wrap;">点击「刷新」查看日志</div>
      </div>
    `;

    panel.querySelector('#btnVerboseAllOn')?.addEventListener('click', () => toggleAllVerbose(true));
    panel.querySelector('#btnVerboseAllOff')?.addEventListener('click', () => toggleAllVerbose(false));
    panel.querySelector('#btnVerboseRefresh')?.addEventListener('click', refreshVerboseLogs);
    panel.querySelector('#btnVerboseExport')?.addEventListener('click', exportVerboseLogs);

    fetchVerboseLogStatus();
  }

  global.initVerboseLogWidget = renderVerboseLogCard;
  global.fetchVerboseLogStatus = fetchVerboseLogStatus;
  global.toggleVerboseModule = toggleVerboseModule;
  global.toggleAllVerbose = toggleAllVerbose;
  global.refreshVerboseLogs = refreshVerboseLogs;
  global.VERBOSE_MODULE_LABELS = MODULE_LABELS;
})(window);
