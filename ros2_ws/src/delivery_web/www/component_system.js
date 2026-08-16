/**
 * Unified Component Window System for AGV Web Sim.
 * OPEN/CLOSED only — no minimized state.
 */
(function (global) {
  const LAYOUT_KEY = "agv.component.layout";
  let zCounter = 40;

  const CSS = `
.widget-card.glass-comp {
  pointer-events: auto; position: absolute;
  display: flex; flex-direction: column;
  background: rgba(15, 23, 42, 0.58);
  backdrop-filter: blur(16px) saturate(1.15);
  -webkit-backdrop-filter: blur(16px) saturate(1.15);
  border: 1px solid rgba(255,255,255,0.12);
  border-radius: 14px;
  box-shadow: 0 8px 32px rgba(0,0,0,0.28);
  color: #e2e8f0;
  overflow: hidden;
  min-width: 120px; min-height: 80px;
}
.widget-card.glass-comp .widget-head {
  display: flex; align-items: center; gap: 8px;
  padding: 8px 10px; cursor: move; user-select: none;
  background: rgba(255,255,255,0.08);
  border-bottom: 1px solid rgba(255,255,255,0.10);
  font-size: 12px; font-weight: 650; letter-spacing: 0.04em;
  color: #f1f5f9; flex-shrink: 0;
}
.widget-card.glass-comp .widget-head .title { flex: 1; text-transform: uppercase; font-size: 11px; }
.widget-card.glass-comp .widget-body {
  flex: 1; min-height: 0; padding: 10px; overflow: auto;
  background: transparent; color: #e2e8f0; font-size: 12px;
}
.widget-card.glass-comp .widget-body .k, .widget-card.glass-comp .tag { color: #94a3b8; }
.widget-card.glass-comp .widget-body .v, .widget-card.glass-comp .widget-body b { color: #f8fafc; }
.widget-card.glass-comp .icon-btn {
  border: none; background: transparent; color: #94a3b8; cursor: pointer;
  width: 24px; height: 24px; border-radius: 6px; font-size: 16px; line-height: 1;
}
.widget-card.glass-comp .icon-btn:hover { background: rgba(255,255,255,0.08); color: #fff; }
.widget-card.glass-comp .resize-e {
  position: absolute; right: 0; top: 28px; bottom: 10px; width: 6px; cursor: ew-resize;
}
.widget-card.glass-comp .resize-s {
  position: absolute; left: 10px; right: 10px; bottom: 0; height: 6px; cursor: ns-resize;
}
.widget-card.glass-comp .resize-se {
  position: absolute; right: 0; bottom: 0; width: 14px; height: 14px; cursor: nwse-resize;
  background: linear-gradient(135deg, transparent 50%, rgba(255,255,255,0.28) 50%);
}
.widget-card.glass-comp[data-closed="1"] { display: none !important; }
.widget-card.glass-comp .bb-chart { background: rgba(2,6,23,0.85); }
.widget-card.glass-comp .bb-note.cut { background: rgba(185,28,28,0.35); color: #fecaca; border-color: rgba(248,113,113,0.35); }
.widget-card.glass-comp .bb-note.warn { background: rgba(180,83,9,0.35); color: #fde68a; border-color: rgba(251,191,36,0.35); }
.widget-card.glass-comp .bb-note.ok { background: rgba(4,120,87,0.3); color: #a7f3d0; border-color: rgba(52,211,153,0.35); }
.widget-card.glass-comp .bb-state { background: rgba(255,255,255,0.06); }
.widget-card.glass-comp input,
.widget-card.glass-comp select,
.widget-card.glass-comp textarea {
  color: #0f172a !important;
  background: rgba(255,255,255,0.92) !important;
}
.pick-group { margin: 10px 0 4px; font-size: 11px; font-weight: 700; color: #64748b; letter-spacing: 0.06em; text-transform: uppercase; }
`;

  function loadRaw() {
    try {
      return JSON.parse(localStorage.getItem(LAYOUT_KEY) || "{}") || {};
    } catch {
      return {};
    }
  }

  function saveRaw(obj) {
    try {
      localStorage.setItem(LAYOUT_KEY, JSON.stringify(obj));
    } catch (_) {}
  }

  function clampLayout(id, def, layout, vp) {
    const minW = Math.max(120, def.minW || def.w || 180);
    const minH = Math.max(80, def.minH || def.h || 120);
    let w = Number(layout.width ?? def.w ?? 240);
    let h = Number(layout.height ?? def.h ?? 180);
    let x = Number(layout.x ?? def.x ?? 16);
    let y = Number(layout.y ?? def.y ?? 64);
    if (!Number.isFinite(w) || w < minW) w = minW;
    if (!Number.isFinite(h) || h < minH) h = minH;
    if (!Number.isFinite(x)) x = def.x || 16;
    if (!Number.isFinite(y)) y = def.y || 64;
    const maxX = Math.max(0, vp.w - 80);
    const maxY = Math.max(0, vp.h - 36);
    x = Math.min(Math.max(0, x), maxX);
    y = Math.min(Math.max(0, y), maxY);
    if (x + w > vp.w) x = Math.max(0, vp.w - w);
    if (y + 36 > vp.h) y = Math.max(0, vp.h - 36);
    return { x, y, width: w, height: h, open: !!layout.open, minW, minH };
  }

  function create(opts) {
    const layer = opts.layerEl;
    const registry = opts.registry || {};
    const getViewport = opts.getViewport || (() => ({ w: window.innerWidth, h: window.innerHeight }));
    const cards = new Map();
    let layoutStore = loadRaw();

    function persistCard(id) {
      const card = cards.get(id);
      if (!card) return;
      const def = registry[id];
      if (!def) return;
      layoutStore[id] = {
        x: card.offsetLeft,
        y: card.offsetTop,
        width: card.offsetWidth,
        height: card.offsetHeight,
        open: card.dataset.closed !== "1" && card.style.display !== "none",
      };
      // strip legacy
      delete layoutStore[id].minimized;
      saveRaw(layoutStore);
    }

    function bringToFront(card) {
      zCounter += 1;
      card.style.zIndex = String(zCounter);
    }

    function enableInteract(card, id, minW, minH) {
      const head = card.querySelector(".widget-head");
      const handles = [
        { el: card.querySelector(".resize-e"), mode: "e" },
        { el: card.querySelector(".resize-s"), mode: "s" },
        { el: card.querySelector(".resize-se"), mode: "se" },
      ];
      let mode = null;
      let ox = 0, oy = 0, ol = 0, ot = 0, ow = 0, oh = 0;

      card.addEventListener("pointerdown", () => bringToFront(card));

      head.onpointerdown = (e) => {
        if (e.target.closest(".icon-btn")) return;
        mode = "drag";
        ol = card.offsetLeft;
        ot = card.offsetTop;
        ox = e.clientX;
        oy = e.clientY;
        head.setPointerCapture(e.pointerId);
        bringToFront(card);
      };

      handles.forEach(({ el, mode: m }) => {
        if (!el) return;
        el.onpointerdown = (e) => {
          mode = m;
          ow = card.offsetWidth;
          oh = card.offsetHeight;
          ol = card.offsetLeft;
          ot = card.offsetTop;
          ox = e.clientX;
          oy = e.clientY;
          el.setPointerCapture(e.pointerId);
          e.stopPropagation();
          bringToFront(card);
        };
      });

      const onMove = (e) => {
        if (!mode) return;
        const vp = getViewport();
        if (mode === "drag") {
          let nx = ol + e.clientX - ox;
          let ny = ot + e.clientY - oy;
          nx = Math.min(Math.max(-card.offsetWidth + 80, nx), vp.w - 40);
          ny = Math.min(Math.max(0, ny), vp.h - 36);
          card.style.left = nx + "px";
          card.style.top = ny + "px";
        } else {
          let nw = ow;
          let nh = oh;
          if (mode === "e" || mode === "se") nw = ow + (e.clientX - ox);
          if (mode === "s" || mode === "se") nh = oh + (e.clientY - oy);
          nw = Math.max(minW, nw);
          nh = Math.max(minH, nh);
          card.style.width = nw + "px";
          card.style.height = nh + "px";
          card._onResize?.();
        }
      };
      const onUp = () => {
        if (!mode) return;
        mode = null;
        persistCard(id);
        card._onResize?.();
      };

      head.onpointermove = onMove;
      head.onpointerup = onUp;
      handles.forEach(({ el }) => {
        if (!el) return;
        el.onpointermove = onMove;
        el.onpointerup = onUp;
      });
    }

    function ensureCard(id) {
      if (cards.has(id)) return cards.get(id);
      const def = registry[id];
      if (!def) return null;
      const vp = getViewport();
      const saved = layoutStore[id] || {};
      const L = clampLayout(id, def, saved, vp);
      const card = document.createElement("div");
      card.className = "widget-card glass-comp";
      card.dataset.widgetId = id;
      if (def.scrollBody) card.dataset.scrollBody = "1";
      card.style.left = L.x + "px";
      card.style.top = L.y + "px";
      card.style.width = L.width + "px";
      card.style.height = L.height + "px";
      card.innerHTML = `
        <div class="widget-head"><span class="title">${def.title}</span>
          <button class="icon-btn close" type="button" title="关闭">×</button></div>
        <div class="widget-body"></div>
        <div class="resize-e"></div><div class="resize-s"></div><div class="resize-se"></div>`;
      layer.appendChild(card);
      card.querySelector(".close").onclick = () => manager.close(id);
      enableInteract(card, id, L.minW, L.minH);
      def.init(card);
      cards.set(id, card);
      if (!L.open && saved.open === false) {
        card.dataset.closed = "1";
      }
      return card;
    }

    const manager = {
      open(id) {
        const def = registry[id];
        if (!def) return null;
        let card = cards.get(id);
        if (!card) {
          layoutStore[id] = { ...(layoutStore[id] || {}), open: true };
          card = ensureCard(id);
        }
        if (!card) return null;
        const vp = getViewport();
        const L = clampLayout(id, def, {
          x: card.offsetLeft,
          y: card.offsetTop,
          width: Math.max(card.offsetWidth, def.minW || def.w),
          height: Math.max(card.offsetHeight, def.minH || def.h),
          open: true,
        }, vp);
        card.style.left = L.x + "px";
        card.style.top = L.y + "px";
        card.style.width = L.width + "px";
        card.style.height = L.height + "px";
        card.dataset.closed = "0";
        card.removeAttribute("data-closed");
        card.style.display = "flex";
        bringToFront(card);
        persistCard(id);
        return card;
      },
      close(id) {
        const card = cards.get(id);
        if (!card) return;
        card.dataset.closed = "1";
        card.style.display = "none";
        persistCard(id);
      },
      isOpen(id) {
        const card = cards.get(id);
        return !!(card && card.dataset.closed !== "1" && card.style.display !== "none");
      },
      bringToFront,
      openMany(ids) {
        (ids || []).forEach((id) => manager.open(id));
      },
      closeAll() {
        cards.forEach((_, id) => manager.close(id));
      },
      saveLayout() {
        cards.forEach((_, id) => persistCard(id));
      },
      loadLayout() {
        layoutStore = loadRaw();
        Object.keys(registry).forEach((id) => {
          const s = layoutStore[id];
          if (s && s.open) manager.open(id);
        });
      },
      getOpenIds() {
        return [...cards.keys()].filter((id) => manager.isOpen(id));
      },
      applySnap(snap) {
        cards.forEach((card) => {
          if (card.dataset.closed === "1" || card.style.display === "none") return;
          if (typeof card._onSnap === "function") card._onSnap(snap);
        });
      },
      ensureCard,
      registry,
    };

  // patch open to clear display:none
  const _open = manager.open;
  manager.open = function (id) {
    const card = _open(id);
    if (card) {
      card.style.display = "flex";
      card.dataset.closed = "0";
      // remove closed attr entirely so !important CSS cannot hide it
      card.removeAttribute("data-closed");
    }
    return card;
  };
  return manager;
  }

  global.AgvComponentSystem = { create, CSS, LAYOUT_KEY };
})(window);
