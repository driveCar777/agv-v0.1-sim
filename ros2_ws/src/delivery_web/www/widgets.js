/**
 * AGV Web Widget Manager — draggable glassmorphism cards on map canvas.
 */
(function (global) {
  'use strict';

  const STORAGE_KEY = 'agv_widget_layout_v06';
  const LEGACY_STORAGE_KEY = 'agv_widget_layout_v05';
  const MIN_W = 200;
  const MIN_H = 120;

  const RESIZE_HANDLES = [
    { dir: 'n', cursor: 'ns-resize', cls: 'resize-n' },
    { dir: 's', cursor: 'ns-resize', cls: 'resize-s' },
    { dir: 'e', cursor: 'ew-resize', cls: 'resize-e' },
    { dir: 'w', cursor: 'ew-resize', cls: 'resize-w' },
    { dir: 'ne', cursor: 'nesw-resize', cls: 'resize-ne' },
    { dir: 'nw', cursor: 'nwse-resize', cls: 'resize-nw' },
    { dir: 'se', cursor: 'nwse-resize', cls: 'resize-se' },
    { dir: 'sw', cursor: 'nesw-resize', cls: 'resize-sw' },
  ];

  class WidgetManager {
    constructor(containerId) {
      this.container = document.getElementById(containerId);
      this.widgets = new Map();
      this.zIndex = 200;
      this.registry = {};
      this._loadRegistry();
    }

    _loadRegistry() {
      if (global.WIDGET_REGISTRY) {
        this.registry = global.WIDGET_REGISTRY;
      }
    }

    isOpen(id) {
      return this.widgets.has(id);
    }

    listAvailable() {
      this._loadRegistry();
      return Object.entries(this.registry).map(([id, cfg]) => ({
        id,
        title: cfg.title,
        open: this.isOpen(id),
      }));
    }

    addWidget(id, opts = {}) {
      this._loadRegistry();
      if (this.widgets.has(id)) return;
      const cfg = this.registry[id];
      if (!cfg) return;

      const card = this._createCard(id, cfg);
      this.container.appendChild(card);
      this.widgets.set(id, card);

      const body = card.querySelector('.widget-body');
      if (cfg.adopt) {
        const el = typeof cfg.adopt === 'string' ? document.querySelector(cfg.adopt) : cfg.adopt;
        if (el) {
          el.style.display = '';
          el.classList.add('widget-adopted');
          body.appendChild(el);
        }
      } else if (cfg.bodyHtml) {
        body.innerHTML = cfg.bodyHtml;
      }

      this._makeDraggable(card);
      this._makeResizable(card);
      this._restoreLayout(id, card, cfg, opts);

      if (typeof cfg.init === 'function') {
        try {
          cfg.init(card, this);
        } catch (e) {
          console.error('widget init failed', id, e);
        }
      }
      if (!this._bulkRestore) {
        this._saveLayout();
      }
      this._refreshPickerIfOpen();
    }

    removeWidget(id) {
      const card = this.widgets.get(id);
      if (!card) return;
      const cfg = this.registry[id];
      if (cfg && typeof cfg.destroy === 'function') {
        try {
          cfg.destroy(card);
        } catch (e) {
          console.error('widget destroy', id, e);
        }
      }
      card.remove();
      this.widgets.delete(id);
      this._saveLayout();
      this._refreshPickerIfOpen();
      if (typeof global.onWidgetClosed === 'function') global.onWidgetClosed(id);
    }

    bringToFront(card) {
      card.style.zIndex = String(++this.zIndex);
    }

    _createCard(id, cfg) {
      const card = document.createElement('div');
      card.className = 'widget-card';
      card.dataset.widgetId = id;
      const minW = cfg.minW || MIN_W;
      const minH = cfg.minH || MIN_H;
      card.dataset.minW = String(minW);
      card.dataset.minH = String(minH);
      const w = cfg.defaultW || 300;
      const h = cfg.defaultH || 220;
      card.style.width = w + 'px';
      if (h) card.style.height = h + 'px';

      const handlesHtml = RESIZE_HANDLES.map(
        (h) => `<div class="widget-resize-handle ${h.cls}" data-dir="${h.dir}" title="调整大小"></div>`
      ).join('');

      card.innerHTML = `
        <div class="widget-header">
          <span class="widget-title">${cfg.title || id}</span>
          <div class="widget-actions">
            <button type="button" class="widget-minimize" title="最小化">—</button>
            <button type="button" class="widget-close" title="关闭">×</button>
          </div>
        </div>
        <div class="widget-body"></div>
        ${handlesHtml}
      `;

      const offset = this.widgets.size * 24;
      card.style.left = Math.max(12, (this.container.clientWidth || 800) - w - 20 - offset) + 'px';
      card.style.top = (64 + offset) + 'px';

      card.querySelector('.widget-close').addEventListener('click', (e) => {
        e.stopPropagation();
        this.removeWidget(id);
      });

      const minBtn = card.querySelector('.widget-minimize');
      minBtn.addEventListener('click', (e) => {
        e.stopPropagation();
        card.classList.toggle('minimized');
        this._saveLayout();
      });

      card.addEventListener('mousedown', () => this.bringToFront(card));
      card.querySelector('.widget-header').addEventListener('dblclick', () => {
        card.classList.toggle('high-opacity');
      });

      return card;
    }

    _makeDraggable(card) {
      const header = card.querySelector('.widget-header');
      let dragging = false;
      let sx, sy, ox, oy;

      header.addEventListener('mousedown', (e) => {
        if (e.target.closest('button')) return;
        dragging = true;
        card.classList.add('dragging');
        sx = e.clientX;
        sy = e.clientY;
        ox = card.offsetLeft;
        oy = card.offsetTop;
        e.preventDefault();
      });

      const onMove = (e) => {
        if (!dragging) return;
        const cw = this.container.clientWidth;
        const ch = this.container.clientHeight;
        let nx = ox + e.clientX - sx;
        let ny = oy + e.clientY - sy;
        nx = Math.max(0, Math.min(nx, cw - card.offsetWidth));
        ny = Math.max(0, Math.min(ny, ch - 40));
        card.style.left = nx + 'px';
        card.style.top = ny + 'px';
      };

      const onUp = () => {
        if (dragging) {
          dragging = false;
          card.classList.remove('dragging');
          this._scheduleSaveLayout();
        }
      };

      window.addEventListener('mousemove', onMove);
      window.addEventListener('mouseup', onUp);
    }

    _cardMin(card) {
      return {
        w: parseInt(card.dataset.minW, 10) || MIN_W,
        h: parseInt(card.dataset.minH, 10) || MIN_H,
      };
    }

    _applyResize(card, dir, dx, dy, start) {
      const cw = this.container.clientWidth;
      const ch = this.container.clientHeight;
      const min = this._cardMin(card);
      let { left, top, width, height } = start;
      const right = left + width;
      const bottom = top + height;

      if (dir.includes('e')) width = Math.max(min.w, width + dx);
      if (dir.includes('w')) {
        const nw = Math.max(min.w, width - dx);
        left = right - nw;
        width = nw;
      }
      if (dir.includes('s')) height = Math.max(min.h, height + dy);
      if (dir.includes('n')) {
        const nh = Math.max(min.h, height - dy);
        top = bottom - nh;
        height = nh;
      }

      left = Math.max(0, Math.min(left, cw - min.w));
      top = Math.max(0, Math.min(top, ch - 40));
      width = Math.min(width, cw - left);
      height = Math.min(height, ch - top);

      card.style.left = left + 'px';
      card.style.top = top + 'px';
      card.style.width = width + 'px';
      card.style.height = height + 'px';
    }

    _makeResizable(card) {
      card.querySelectorAll('.widget-resize-handle').forEach((handle) => {
        const dir = handle.dataset.dir;
        let resizing = false;
        let sx, sy;
        let start;

        handle.addEventListener('mousedown', (e) => {
          resizing = true;
          card.classList.add('resizing');
          sx = e.clientX;
          sy = e.clientY;
          start = {
            left: card.offsetLeft,
            top: card.offsetTop,
            width: card.offsetWidth,
            height: card.offsetHeight,
          };
          e.preventDefault();
          e.stopPropagation();
        });

        const onMove = (e) => {
          if (!resizing) return;
          this._applyResize(card, dir, e.clientX - sx, e.clientY - sy, start);
        };

        const onUp = () => {
          if (resizing) {
            resizing = false;
            card.classList.remove('resizing');
            this._scheduleSaveLayout();
          }
        };

        window.addEventListener('mousemove', onMove);
        window.addEventListener('mouseup', onUp);
      });
    }

    _saveLayout() {
      const layout = { v: 2, open: [], widgets: {} };
      this.widgets.forEach((card, id) => {
        layout.open.push(id);
        layout.widgets[id] = {
          x: parseInt(card.style.left, 10) || 0,
          y: parseInt(card.style.top, 10) || 0,
          w: card.offsetWidth,
          h: card.offsetHeight,
          minimized: card.classList.contains('minimized'),
        };
      });
      try {
        localStorage.setItem(STORAGE_KEY, JSON.stringify(layout));
        localStorage.removeItem(LEGACY_STORAGE_KEY);
      } catch (e) {
        /* ignore */
      }
    }

    _scheduleSaveLayout() {
      if (this._saveTimer) clearTimeout(this._saveTimer);
      this._saveTimer = setTimeout(() => {
        this._saveTimer = null;
        this._saveLayout();
      }, 120);
    }

    _readLayout() {
      try {
        const raw = localStorage.getItem(STORAGE_KEY) || localStorage.getItem(LEGACY_STORAGE_KEY);
        return raw ? JSON.parse(raw) : {};
      } catch (e) {
        return {};
      }
    }

    _restoreLayout(id, card, cfg, opts) {
      const layout = this._readLayout();
      const saved = (layout.widgets || {})[id] || opts;
      if (saved && (saved.x != null || saved.y != null)) {
        if (saved.x != null) card.style.left = saved.x + 'px';
        if (saved.y != null) card.style.top = saved.y + 'px';
        if (saved.w) card.style.width = saved.w + 'px';
        if (saved.h) card.style.height = saved.h + 'px';
        if (saved.minimized) card.classList.add('minimized');
      } else if (cfg.defaultX != null) {
        card.style.left = cfg.defaultX + 'px';
        card.style.top = (cfg.defaultY || 64) + 'px';
      }
      this._clampCard(card);
    }

    _clampCard(card) {
      const cw = this.container.clientWidth || window.innerWidth;
      const ch = this.container.clientHeight || window.innerHeight;
      const min = this._cardMin(card);
      let left = parseInt(card.style.left, 10) || 0;
      let top = parseInt(card.style.top, 10) || 0;
      const width = card.offsetWidth || parseInt(card.style.width, 10) || min.w;
      const height = card.offsetHeight || parseInt(card.style.height, 10) || min.h;
      left = Math.max(0, Math.min(left, Math.max(0, cw - min.w)));
      top = Math.max(0, Math.min(top, Math.max(0, ch - 40)));
      card.style.left = left + 'px';
      card.style.top = top + 'px';
      card.style.width = Math.min(width, Math.max(min.w, cw - left)) + 'px';
      card.style.height = Math.min(height, Math.max(min.h, ch - top)) + 'px';
    }

    restoreFromStorage(defaultIds) {
      const layout = this._readLayout();
      const ids = layout.open && layout.open.length ? layout.open : defaultIds;
      this._bulkRestore = true;
      try {
        ids.forEach((id) => {
          if (this.registry[id]) {
            const saved = (layout.widgets || {})[id] || {};
            this.addWidget(id, saved);
          }
        });
      } finally {
        this._bulkRestore = false;
      }
      this._saveLayout();
      requestAnimationFrame(() => {
        this.widgets.forEach((card) => this._clampCard(card));
        this._saveLayout();
      });
    }

    _pickerOpen() {
      const panel = document.getElementById('widgetPicker');
      return panel && panel.classList.contains('open');
    }

    _refreshPickerIfOpen() {
      if (this._pickerOpen()) this.renderPicker();
    }

    _widgetIcon(title) {
      const m = String(title || '').match(/^(\S+)/);
      return m ? m[1] : '◻';
    }

    _widgetName(title) {
      return String(title || '').replace(/^\S+\s*/, '') || title;
    }

    renderPicker() {
      this._loadRegistry();
      const list = document.getElementById('widgetPickerList');
      if (!list) return;

      const items = this.listAvailable();
      if (!items.length) {
        list.innerHTML = '<div class="widget-picker-empty">暂无可用组件（注册表为空）</div>';
        return;
      }

      list.innerHTML = items
        .map((item) => {
          const isAdded = item.open;
          const btnText = isAdded ? '− 移除' : '+ 添加';
          const btnClass = isAdded ? 'widget-remove-btn' : 'widget-add-btn';
          const icon = this._widgetIcon(item.title);
          const name = this._widgetName(item.title);
          return `
            <div class="widget-list-item" data-widget-id="${item.id}">
              <div class="widget-list-item-info">
                <div class="widget-list-item-icon">${icon}</div>
                <div>
                  <div class="widget-list-item-name">${name}</div>
                </div>
              </div>
              <button type="button" class="${btnClass}" data-widget-id="${item.id}">${btnText}</button>
            </div>`;
        })
        .join('');

      list.querySelectorAll('button[data-widget-id]').forEach((btn) => {
        btn.addEventListener('click', (e) => {
          e.stopPropagation();
          const wid = btn.dataset.widgetId;
          if (this.widgets.has(wid)) {
            this.removeWidget(wid);
          } else {
            this.addWidget(wid);
          }
        });
      });
    }

    openPicker() {
      const panel = document.getElementById('widgetPicker');
      const backdrop = document.getElementById('widgetPickerBackdrop');
      if (!panel) return;
      this.renderPicker();
      panel.classList.add('open');
      if (backdrop) backdrop.classList.add('open');
    }

    closePicker() {
      const panel = document.getElementById('widgetPicker');
      const backdrop = document.getElementById('widgetPickerBackdrop');
      if (panel) panel.classList.remove('open');
      if (backdrop) backdrop.classList.remove('open');
    }
  }

  global.WidgetManager = WidgetManager;
})(window);
