/**
 * 第三人称主视口
 * - 跟随 ON：默认车尾正后、对齐车头；左键左右拖改视角；3s 无操作回弹到改前偏移
 * - 跟随 OFF：左键拖平移；不回弹视角
 * - 滚轮：逆时针贴近(≥2.8m)；顺时针拉远并抬到头顶俯视(≤100m)
 */
(function (global) {
  const MIN_DIST = 2.8;
  const MAX_DIST = 100.0;
  const ANGLE_IDLE_MS = 3000;
  const RESTORE_SPEED = 4.5;

  function createAgvMesh() {
    const g = new THREE.Group();
    g.name = "AMB-150";

    const bodyMat = new THREE.MeshStandardMaterial({
      color: 0xf4f4f4,
      metalness: 0.28,
      roughness: 0.48,
    });
    const accent = new THREE.MeshStandardMaterial({
      color: 0xe5364a,
      metalness: 0.15,
      roughness: 0.35,
    });
    const dark = new THREE.MeshStandardMaterial({
      color: 0x1a1a1a,
      metalness: 0.45,
      roughness: 0.55,
    });
    const glass = new THREE.MeshStandardMaterial({
      color: 0x1e293b,
      metalness: 0.6,
      roughness: 0.2,
      transparent: true,
      opacity: 0.85,
    });

    const base = new THREE.Mesh(new THREE.BoxGeometry(1.05, 0.22, 0.72), bodyMat);
    base.position.y = 0.18;
    base.castShadow = true;
    g.add(base);

    const skirt = new THREE.Mesh(new THREE.BoxGeometry(1.08, 0.06, 0.76), dark);
    skirt.position.y = 0.06;
    g.add(skirt);

    const deck = new THREE.Mesh(new THREE.BoxGeometry(0.92, 0.16, 0.58), bodyMat);
    deck.position.y = 0.36;
    deck.castShadow = true;
    g.add(deck);

    const cover = new THREE.Mesh(new THREE.BoxGeometry(0.78, 0.08, 0.48), bodyMat);
    cover.position.y = 0.48;
    g.add(cover);

    const stripe = new THREE.Mesh(new THREE.BoxGeometry(1.06, 0.035, 0.06), accent);
    stripe.position.set(0, 0.3, 0.34);
    g.add(stripe);
    const stripe2 = new THREE.Mesh(new THREE.BoxGeometry(1.06, 0.035, 0.06), accent);
    stripe2.position.set(0, 0.3, -0.34);
    g.add(stripe2);

    const nose = new THREE.Mesh(new THREE.BoxGeometry(0.14, 0.1, 0.36), accent);
    nose.position.set(0.56, 0.26, 0);
    g.add(nose);

    const lampMat = new THREE.MeshStandardMaterial({
      color: 0xfde68a,
      emissive: 0xf59e0b,
      emissiveIntensity: 0.55,
    });
    [-0.22, 0.22].forEach((z) => {
      const lamp = new THREE.Mesh(new THREE.BoxGeometry(0.04, 0.04, 0.08), lampMat);
      lamp.position.set(0.62, 0.28, z);
      g.add(lamp);
    });

    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.03, 0.28, 10), dark);
    pole.position.set(-0.28, 0.62, 0.18);
    g.add(pole);
    const beacon = new THREE.Mesh(
      new THREE.SphereGeometry(0.045, 12, 12),
      new THREE.MeshStandardMaterial({ color: 0x22c55e, emissive: 0x16a34a, emissiveIntensity: 0.7 })
    );
    beacon.position.set(-0.28, 0.78, 0.18);
    g.add(beacon);

    const lidGeo = new THREE.CylinderGeometry(0.07, 0.07, 0.09, 14);
    const lidMat = new THREE.MeshStandardMaterial({ color: 0x0f2744, metalness: 0.55, roughness: 0.25 });
    const lf = new THREE.Mesh(lidGeo, lidMat);
    lf.position.set(0.38, 0.56, 0.24);
    g.add(lf);
    const lr = new THREE.Mesh(lidGeo, lidMat);
    lr.position.set(-0.38, 0.56, -0.24);
    g.add(lr);

    const wheelGeo = new THREE.CylinderGeometry(0.13, 0.13, 0.09, 18);
    [
      [0.34, 0.13, 0.4],
      [0.34, 0.13, -0.4],
      [-0.34, 0.13, 0.4],
      [-0.34, 0.13, -0.4],
    ].forEach(([x, y, z]) => {
      const w = new THREE.Mesh(wheelGeo, dark);
      w.rotation.z = Math.PI / 2;
      w.position.set(x, y, z);
      g.add(w);
    });

    const win = new THREE.Mesh(new THREE.BoxGeometry(0.35, 0.08, 0.02), glass);
    win.position.set(0.1, 0.4, 0.37);
    g.add(win);

    const plate = new THREE.Mesh(new THREE.BoxGeometry(0.28, 0.06, 0.01), accent);
    plate.position.set(-0.2, 0.42, 0.37);
    g.add(plate);

    return g;
  }

  function lerpAngle(a, b, t) {
    let d = ((b - a + Math.PI) % (Math.PI * 2)) - Math.PI;
    if (d < -Math.PI) d += Math.PI * 2;
    return a + d * t;
  }

  class Sim3DView {
    constructor(canvas) {
      this.canvas = canvas;
      this.renderer = new THREE.WebGLRenderer({
        canvas,
        antialias: true,
        alpha: false,
      });
      this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
      this.renderer.setClearColor(0x0b1220, 1);
      this.renderer.shadowMap.enabled = true;

      this.scene = new THREE.Scene();
      this.scene.fog = new THREE.FogExp2(0x0b1220, 0.012);
      this.camera = new THREE.PerspectiveCamera(52, 1, 0.12, 280);

      this.follow = true;
      this.camDist = 8.5;
      this.orbitYawOffset = 0; // 0 = 车尾正后，对齐车头前进方向
      this.orbitPitch = 0.72;
      this.panX = 0;
      this.panY = 0;

      this._homeYawOffset = 0;
      this._homePitch = 0.72;
      this._angleDirty = false;
      this._lastAngleAt = 0;
      this._restoring = false;

      this.ambient = new THREE.AmbientLight(0xffffff, 0.5);
      this.scene.add(this.ambient);
      this.hemi = new THREE.HemisphereLight(0xb8c8e8, 0x1a2030, 0.45);
      this.scene.add(this.hemi);
      this.dir = new THREE.DirectionalLight(0xffffff, 0.9);
      this.dir.position.set(10, 22, 8);
      this.dir.castShadow = true;
      this.scene.add(this.dir);

      const ground = new THREE.Mesh(
        new THREE.PlaneGeometry(260, 260),
        new THREE.MeshStandardMaterial({ color: 0x101826, roughness: 0.96, metalness: 0 })
      );
      ground.rotation.x = -Math.PI / 2;
      ground.receiveShadow = true;
      this.scene.add(ground);

      this.grid = new THREE.GridHelper(200, 80, 0x2a3a55, 0x182033);
      this.grid.position.y = 0.015;
      this.scene.add(this.grid);

      this.agv = createAgvMesh();
      this.scene.add(this.agv);

      this.cloudGeom = new THREE.BufferGeometry();
      this.cloudMat = new THREE.PointsMaterial({
        size: 0.07,
        color: 0x94a3b8,
        sizeAttenuation: true,
        transparent: true,
        opacity: 0.88,
      });
      this.cloud = new THREE.Points(this.cloudGeom, this.cloudMat);
      this.scene.add(this.cloud);

      this.liveGeom = new THREE.BufferGeometry();
      this.liveMat = new THREE.PointsMaterial({
        size: 0.11,
        color: 0xfbbf24,
        sizeAttenuation: true,
      });
      this.live = new THREE.Points(this.liveGeom, this.liveMat);
      this.scene.add(this.live);

      this.band = null;
      this.pathColor = "#3B82F6";
      this.dragging = false;
      this.lastX = 0;
      this.lastY = 0;
      this.pose = { x: 0, y: 0, yaw: 0 };
      this._listeners = { followChange: [] };

      this._bindInput();
      this._resize();
      window.addEventListener("resize", () => this._resize());
      this._lastFrame = performance.now();
      this._loop();
    }

    onFollowChange(fn) {
      this._listeners.followChange.push(fn);
    }

    /** 滚轮远近时轻微联动俯仰（仅未手改视角时） */
    _nudgePitchFromZoom() {
      if (this._angleDirty || this._restoring) return;
      const t = (this.camDist - MIN_DIST) / (MAX_DIST - MIN_DIST);
      const u = Math.min(1, Math.max(0, t));
      const target = 0.55 + u * (1.45 - 0.55);
      this.orbitPitch += (target - this.orbitPitch) * 0.35;
      this._homePitch = this.orbitPitch;
    }

    setFollow(on) {
      const next = !!on;
      if (next === this.follow) return;
      this.follow = next;
      if (this.follow) {
        this.panX = 0;
        this.panY = 0;
        this.orbitYawOffset = 0;
        this.orbitPitch = 0.72;
        this._homeYawOffset = 0;
        this._homePitch = 0.72;
        this._angleDirty = false;
        this._restoring = false;
      } else {
        this._angleDirty = false;
        this._restoring = false;
      }
      this._listeners.followChange.forEach((fn) => {
        try {
          fn(this.follow);
        } catch (_) {}
      });
    }

    toggleFollow() {
      this.setFollow(!this.follow);
      return this.follow;
    }

    _markAngleUserEdit() {
      if (!this.follow) return; // 非跟随不回弹
      if (!this._angleDirty && !this._restoring) {
        this._homeYawOffset = this.orbitYawOffset;
        this._homePitch = this.orbitPitch;
      }
      this._angleDirty = true;
      this._restoring = false;
      this._lastAngleAt = performance.now();
    }

    _bindInput() {
      const el = this.canvas;

      el.addEventListener(
        "wheel",
        (e) => {
          e.preventDefault();
          // 逆时针贴近 / 顺时针拉远俯视
          const steps = Math.max(-12, Math.min(12, e.deltaY));
          const factor = Math.exp(steps * 0.0024);
          this.camDist = Math.min(MAX_DIST, Math.max(MIN_DIST, this.camDist * factor));
          this._nudgePitchFromZoom();
        },
        { passive: false }
      );

      el.addEventListener("pointerdown", (e) => {
        if (e.button !== 0) return;
        this.dragging = true;
        this.lastX = e.clientX;
        this.lastY = e.clientY;
        el.setPointerCapture(e.pointerId);
      });
      el.addEventListener("pointerup", (e) => {
        if (e.button === 0) this.dragging = false;
      });
      el.addEventListener("pointercancel", () => {
        this.dragging = false;
      });
      el.addEventListener("pointermove", (e) => {
        if (!this.dragging) return;
        const dx = e.clientX - this.lastX;
        const dy = e.clientY - this.lastY;
        this.lastX = e.clientX;
        this.lastY = e.clientY;

        // 上下：上=抬到俯视；下=翻转感；斜向同时改左右+俯仰
        if (Math.abs(dy) > 0.01) {
          if (this.follow) this._markAngleUserEdit();
          // 上拖=翻转感；下拖=抬到俯视（与上一版对调）
          this.orbitPitch = Math.min(1.55, Math.max(-0.55, this.orbitPitch + dy * 0.005));
        }

        if (this.follow) {
          if (Math.abs(dx) > 0.01) {
            this._markAngleUserEdit();
            this.orbitYawOffset -= dx * 0.006;
          }
        } else if (Math.abs(dx) > 0.01) {
          const panScale = this.camDist * 0.0018;
          const backAng = this.pose.yaw + Math.PI + this.orbitYawOffset;
          const rightX = Math.cos(backAng + Math.PI / 2);
          const rightY = Math.sin(backAng + Math.PI / 2);
          this.panX += (-dx * panScale) * rightX;
          this.panY += (-dx * panScale) * rightY;
        }
      });
    }

    _resize() {
      const parent = this.canvas.parentElement || this.canvas;
      const w = parent.clientWidth || window.innerWidth;
      const h = parent.clientHeight || window.innerHeight;
      this.renderer.setSize(w, h, false);
      this.camera.aspect = w / Math.max(1, h);
      this.camera.updateProjectionMatrix();
    }

    setPathColor(hex) {
      this.pathColor = hex || "#3B82F6";
    }

    setPose(x, y, yaw) {
      this.pose = { x, y, yaw };
      this.agv.position.set(x, 0, -y);
      this.agv.rotation.y = yaw;
    }

    setSurroundCloud(points, isLive) {
      const arr = points || [];
      const n = arr.length;
      const pos = new Float32Array(n * 3);
      for (let i = 0; i < n; i++) {
        const p = arr[i];
        pos[i * 3] = p.x;
        pos[i * 3 + 1] = p.z != null ? p.z : 0.08;
        pos[i * 3 + 2] = -p.y;
      }
      const geom = isLive ? this.liveGeom : this.cloudGeom;
      geom.setAttribute("position", new THREE.BufferAttribute(pos, 3));
      geom.computeBoundingSphere();
      this.cloud.visible = !isLive;
      this.live.visible = !!isLive;
    }

    setLiveLidar(points) {
      const arr = points || [];
      const n = arr.length;
      const pos = new Float32Array(n * 3);
      for (let i = 0; i < n; i++) {
        const p = arr[i];
        pos[i * 3] = p.x;
        pos[i * 3 + 1] = p.z != null ? p.z : 0.35;
        pos[i * 3 + 2] = -p.y;
      }
      this.liveGeom.setAttribute("position", new THREE.BufferAttribute(pos, 3));
      this.liveGeom.computeBoundingSphere();
      this.live.visible = n > 0;
    }

    setGuideBand(path, colorHex, opts) {
      if (this.band) {
        this.scene.remove(this.band);
        this.band.geometry.dispose();
        this.band.material.dispose();
        this.band = null;
      }
      // STEP 3F: prefer PhysicalTrajectoryCorridor edges (footprint⊕margin)
      const o = opts || {};
      const status = String(o.status || "").toUpperCase();
      const preferBlue = !!o.prefer_blue;
      let colorHexOut = colorHex || "#3B82F6";
      if (!preferBlue) {
        if (status === "VALID" && o.soft_risk) colorHexOut = "#EAB308";
        else if (status === "VALID") colorHexOut = "#22C55E";
        else if (status === "INVALID") colorHexOut = "#EF4444";
        else if (status === "UNKNOWN" || status === "STALE") colorHexOut = "#94A3B8";
      } else {
        // Main map: keep blue/cyan active band; only invalidate → red
        if (status === "INVALID") colorHexOut = "#EF4444";
        else if (status === "VALID" && o.soft_risk) colorHexOut = "#60A5FA";
      }

      let left = o.left_edge || null;
      let right = o.right_edge || null;
      const halfW = o.half_width_m != null ? Number(o.half_width_m) : null;

      // Fallback: centerline + yaw/path-tangent half-width (= vehicle footprint half + margin)
      if ((!left || !right) && path && path.length >= 2) {
        const filtered = [path[0]];
        for (let i = 1; i < path.length; i++) {
          const a = filtered[filtered.length - 1];
          const b = path[i];
          if (Math.hypot(b.x - a.x, b.y - a.y) >= 0.02) filtered.push(b);
        }
        if (filtered.length < 2) return;
        path = filtered;
        const hw = halfW != null && halfW > 0.05 ? halfW : 0.275 + 0.18; // 0.5*W + margins
        left = [];
        right = [];
        for (let i = 0; i < path.length; i++) {
          const p = path[i];
          let nx, ny;
          if (p.yaw != null) {
            nx = -Math.sin(p.yaw);
            ny = Math.cos(p.yaw);
          } else {
            const n = path[Math.min(path.length - 1, i + 1)];
            const dx = n.x - p.x;
            const dy = n.y - p.y;
            const len = Math.hypot(dx, dy) || 1;
            nx = -dy / len;
            ny = dx / len;
          }
          left.push({ x: p.x + nx * hw, y: p.y + ny * hw });
          right.push({ x: p.x - nx * hw, y: p.y - ny * hw });
        }
      }
      if (!left || !right || left.length < 2 || right.length < 2) return;
      const nPts = Math.min(left.length, right.length);
      const color = new THREE.Color(colorHexOut);
      const positions = [];
      const indices = [];
      for (let i = 0; i < nPts; i++) {
        const y = 0.08;
        positions.push(left[i].x, y, -left[i].y);
        positions.push(right[i].x, y, -right[i].y);
      }
      for (let i = 0; i < nPts - 1; i++) {
        const a = i * 2;
        indices.push(a, a + 1, a + 2, a + 1, a + 3, a + 2);
      }
      const geo = new THREE.BufferGeometry();
      geo.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
      geo.setIndex(indices);
      geo.computeVertexNormals();
      const mat = new THREE.MeshBasicMaterial({
        color,
        transparent: true,
        opacity: 0.55,
        side: THREE.DoubleSide,
        depthWrite: false,
      });
      this.band = new THREE.Mesh(geo, mat);
      this.band.renderOrder = 2; // selected local above global reference
      this.scene.add(this.band);
    }

    _clearLayerGroup(name) {
      const g = this[name];
      if (!g) return;
      while (g.children.length) {
        const c = g.children.pop();
        c.geometry?.dispose?.();
        if (c.material) {
          if (Array.isArray(c.material)) c.material.forEach((m) => m.dispose?.());
          else c.material.dispose?.();
        }
      }
    }

    _ensureLayerGroup(name) {
      if (!this[name]) {
        this[name] = new THREE.Group();
        this[name].name = name;
        this.scene.add(this[name]);
      }
      this._clearLayerGroup(name);
      return this[name];
    }

    _addLineTo(group, pts, color, y, opacity) {
      if (!pts || pts.length < 2) return;
      const arr = [];
      for (const p of pts) arr.push(p.x, y, -p.y);
      const geo = new THREE.BufferGeometry();
      geo.setAttribute("position", new THREE.Float32BufferAttribute(arr, 3));
      const mat = new THREE.LineBasicMaterial({
        color,
        transparent: true,
        opacity: opacity != null ? opacity : 0.85,
        depthWrite: false,
      });
      const line = new THREE.Line(geo, mat);
      group.add(line);
    }

    _addDashedLineTo(group, pts, color, y, opacity) {
      if (!pts || pts.length < 2) return;
      const arr = [];
      for (const p of pts) arr.push(p.x, y, -p.y);
      const geo = new THREE.BufferGeometry();
      geo.setAttribute("position", new THREE.Float32BufferAttribute(arr, 3));
      const mat = new THREE.LineDashedMaterial({
        color,
        dashSize: 0.22,
        gapSize: 0.14,
        transparent: true,
        opacity: opacity != null ? opacity : 0.85,
        depthWrite: false,
      });
      const line = new THREE.Line(geo, mat);
      line.computeLineDistances();
      group.add(line);
    }

    _splitPosesAtS(poses, sCut) {
      if (sCut == null || !Number.isFinite(Number(sCut))) {
        return { before: poses || [], after: [] };
      }
      const cut = Number(sCut);
      const before = [];
      const after = [];
      for (const p of poses || []) {
        const s = p && p.s != null ? Number(p.s) : null;
        if (s == null || s + 1e-6 < cut) before.push(p);
        else after.push(p);
      }
      if (before.length && after.length) after.unshift(before[before.length - 1]);
      return { before, after };
    }

    /** P0-B/C LAYER 1: Global Reference Preview (REFERENCE_ONLY). Color = kinematic status. */
    setGlobalReference(gref) {
      const group = this._ensureLayerGroup("_globalRefGroup");
      if (!gref || !gref.poses || gref.poses.length < 2) return;
      const status = String(gref.status || "").toUpperCase();
      if (status === "GOAL_REACHED" || status === "NO_GLOBAL_PATH" || status === "DISABLED") return;
      const kst = String(gref.kinematic_status || "").toUpperCase();
      const SLATE = 0x64748b;
      const AMBER = 0xd97706;
      const RED = 0xef4444;
      if (kst === "DEGRADED") {
        this._addLineTo(group, gref.poses, AMBER, 0.06, 0.7);
      } else if (kst === "INVALID") {
        const parts = this._splitPosesAtS(gref.poses, gref.first_invalid_distance_m);
        if (parts.before.length >= 2) this._addLineTo(group, parts.before, SLATE, 0.06, 0.45);
        if (parts.after.length >= 2) this._addDashedLineTo(group, parts.after, RED, 0.07, 0.9);
        else if (parts.before.length < 2) this._addDashedLineTo(group, gref.poses, RED, 0.07, 0.9);
      } else {
        // VALID / NOT_VALIDATED: translucent slate — not certified-safe paint
        this._addLineTo(group, gref.poses, SLATE, 0.06, kst === "VALID" ? 0.62 : 0.45);
      }
      if (gref.left_edge && gref.right_edge && gref.left_edge.length >= 2) {
        const edgeCol = kst === "INVALID" ? 0xfca5a5 : kst === "DEGRADED" ? 0xfbbf24 : 0x94a3b8;
        this._addLineTo(group, gref.left_edge, edgeCol, 0.05, 0.28);
        this._addLineTo(group, gref.right_edge, edgeCol, 0.05, 0.28);
      }
      group.renderOrder = 0;
    }

    /** P0-B LAYER 2/3: all local candidates + highlighted selected. */
    setLocalCandidates(layer) {
      const group = this._ensureLayerGroup("_localCandGroup");
      const items = (layer && layer.items) || [];
      items.forEach((c) => {
        const poses = c.poses || c.path || [];
        if (!poses || poses.length < 2) return;
        const selected = !!c.selected;
        const valid = c.valid !== false;
        let col = 0x94a3b8;
        let op = 0.55;
        let y = 0.11;
        if (selected) {
          col = 0x2563eb;
          op = 0.95;
          y = 0.14;
        } else if (valid) {
          col = 0x38bdf8;
          op = 0.7;
        } else {
          col = 0xf87171;
          op = 0.4;
        }
        this._addLineTo(group, poses, col, y, op);
      });
      group.renderOrder = 1;
    }

    /** P1-1: selected Rolling Local Plan (medium horizon). Distinct from MPPI band. */
    setLocalPlan(plan) {
      const group = this._ensureLayerGroup("_localPlanGroup");
      if (!plan || !plan.poses || plan.poses.length < 2) return;
      if (plan.active === false) return;
      this._addLineTo(group, plan.poses, 0x1d4ed8, 0.16, 0.98);
      group.renderOrder = 2;
    }

    _ensureDebugGroup() {
      if (!this._debugGroup) {
        this._debugGroup = new THREE.Group();
        this.scene.add(this._debugGroup);
      }
      while (this._debugGroup.children.length) {
        const c = this._debugGroup.children.pop();
        c.geometry?.dispose?.();
        if (c.material) {
          if (Array.isArray(c.material)) c.material.forEach((m) => m.dispose?.());
          else c.material.dispose?.();
        }
      }
    }

    _addPolyline(pts, color, y = 0.12, linewidth = 2) {
      if (!pts || pts.length < 2) return;
      const arr = [];
      for (const p of pts) arr.push(p.x, y, -p.y);
      const geo = new THREE.BufferGeometry();
      geo.setAttribute("position", new THREE.Float32BufferAttribute(arr, 3));
      const mat = new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.9 });
      const line = new THREE.Line(geo, mat);
      this._debugGroup.add(line);
    }

    _addDisk(x, y, r, color, opacity = 0.25) {
      const geo = new THREE.CircleGeometry(r, 28);
      const mat = new THREE.MeshBasicMaterial({
        color,
        transparent: true,
        opacity,
        side: THREE.DoubleSide,
        depthWrite: false,
      });
      const mesh = new THREE.Mesh(geo, mat);
      mesh.rotation.x = -Math.PI / 2;
      mesh.position.set(x, 0.05, -y);
      this._debugGroup.add(mesh);
    }

    _addSector(x, y, yaw, halfDeg, range, color, forward = true) {
      const steps = 18;
      const base = forward ? yaw : yaw + Math.PI;
      const half = (halfDeg * Math.PI) / 180;
      const shape = new THREE.Shape();
      shape.moveTo(0, 0);
      for (let i = 0; i <= steps; i++) {
        const a = -half + (2 * half * i) / steps;
        const px = Math.cos(base + a) * range;
        const py = Math.sin(base + a) * range;
        // map→shape local: use x,z later
        shape.lineTo(px, py);
      }
      shape.lineTo(0, 0);
      const geo = new THREE.ShapeGeometry(shape);
      const mat = new THREE.MeshBasicMaterial({
        color,
        transparent: true,
        opacity: 0.18,
        side: THREE.DoubleSide,
        depthWrite: false,
      });
      const mesh = new THREE.Mesh(geo, mat);
      mesh.rotation.x = -Math.PI / 2;
      mesh.position.set(x, 0.04, -y);
      // Shape is in map xy; after rotX, need remap: shape (px,py) → three before rot
      // Easier: build BufferGeometry in three coords
      this._debugGroup.remove(mesh);
      geo.dispose();
      mat.dispose();
      const positions = [x, 0.04, -y];
      for (let i = 0; i <= steps; i++) {
        const a = -half + (2 * half * i) / steps;
        const mx = x + Math.cos(base + a) * range;
        const my = y + Math.sin(base + a) * range;
        positions.push(mx, 0.04, -my);
      }
      const idx = [];
      for (let i = 1; i <= steps; i++) idx.push(0, i, i + 1);
      const g2 = new THREE.BufferGeometry();
      g2.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
      g2.setIndex(idx);
      const m2 = new THREE.MeshBasicMaterial({
        color,
        transparent: true,
        opacity: 0.2,
        side: THREE.DoubleSide,
        depthWrite: false,
      });
      this._debugGroup.add(new THREE.Mesh(g2, m2));
    }

    /**
     * Debug overlays. layers flags from UI.
     * Does not affect navigation.
     */
    setDebugOverlay(dbg, pose, layers) {
      this._ensureDebugGroup();
      if (!dbg || !layers || layers.off) return;
      const paths = dbg.paths || {};
      const geom = dbg.geometry || {};
      const radar = dbg.radar || {};
      const x = pose?.x || 0;
      const y = pose?.y || 0;
      const yaw = pose?.angle || 0;

      if (layers.rawPath) this._addPolyline(paths.raw_global, 0xb45309, 0.10);
      if (layers.processedPath) this._addPolyline(paths.processed_global, 0x059669, 0.14);
      if (layers.executedPath) this._addPolyline(paths.executed, 0x3b82f6, 0.18);
      if (layers.actualTrace) {
        const tr = paths.actual_trace || dbg.pose_trace || [];
        this._addPolyline(tr, 0xf59e0b, 0.16);
      }
      if (layers.candidates) {
        const cands = (dbg.candidates || []).slice(0, 5);
        cands.forEach((c, i) => {
          const col = c.selected ? 0x22c55e : i === 0 ? 0x86efac : 0x94a3b8;
          this._addPolyline(c.path || [], col, 0.09 + i * 0.01);
          if (c.collision && c.first_collision) {
            this._addDisk(c.first_collision.x, c.first_collision.y, 0.12, 0xef4444, 0.7);
          }
        });
      }
      if (layers.footprints) {
        this._addDisk(x, y, geom.planner_radius || 0.25, 0xf59e0b, 0.15);
        this._addDisk(x, y, geom.local_radius || 0.24, 0x3b82f6, 0.12);
        this._addDisk(x, y, geom.safety_radius || 0.28, 0xef4444, 0.12);
      }
      if (layers.sectors) {
        this._addSector(x, y, yaw, radar.sector_half_deg || 22, radar.front_stop_m || 0.7, 0xef4444, true);
        this._addSector(x, y, yaw, radar.sector_half_deg || 22, Math.min(radar.front_near || 3, 4), 0xfbbf24, true);
        this._addSector(x, y, yaw, radar.sector_half_deg || 22, radar.rear_stop_m || 0.55, 0x6366f1, false);
      }
      if (layers.lookahead && dbg.controller?.lookahead_point) {
        const p = dbg.controller.lookahead_point;
        this._addDisk(p.x, p.y, 0.08, 0xec4899, 0.8);
      }
      if (layers.futurePreview && dbg.obstacle_preview) {
        const op = dbg.obstacle_preview;
        const pts = (op.preview_poses || []).map((p) => [p.x, p.y]);
        if (pts.length >= 2) this._addPolyline(pts, 0xf97316, 0.12);
        if (op.left_corridor_poses?.length >= 2) {
          this._addPolyline(op.left_corridor_poses.map((p) => [p.x, p.y]), 0x22d3ee, 0.08);
        }
        if (op.right_corridor_poses?.length >= 2) {
          this._addPolyline(op.right_corridor_poses.map((p) => [p.x, p.y]), 0x22d3ee, 0.08);
        }
        if (op.first_collision_distance_m != null && op.collision_pose) {
          const cp = op.collision_pose;
          this._addDisk(cp.x, cp.y, 0.10, 0xea580c, 0.85);
        }
      }
      const fc = dbg.local_planner?.first_collision;
      if (layers.collision && fc) this._addDisk(fc.x, fc.y, 0.14, 0xdc2626, 0.85);

      if (layers.mapEvents) {
        const catColor = {
          SAFETY: 0xef4444,
          OBSTACLE: 0xf97316,
          REVERSE: 0x8b5cf6,
          RECOVERY: 0xd97706,
          ACCEL: 0x22c55e,
          DECEL: 0xf59e0b,
          TURN: 0x3b82f6,
          REPLAN: 0x06b6d4,
          STOP: 0xb91c1c,
          ERROR: 0xdc2626,
          PLAN: 0x10b981,
        };
        this._mapEventHits = [];
        (dbg.map_events || []).forEach((ev) => {
          if (ev.x == null || ev.y == null) return;
          const col = catColor[ev.type || ev.category] || 0x64748b;
          this._addDisk(ev.x, ev.y, 0.11, col, 0.85);
          this._mapEventHits.push(ev);
        });
      }
    }

    /**
     * Pick nearest map event near map xy (for UI popup).
     */
    pickMapEvent(mx, my, radius = 0.45) {
      const hits = this._mapEventHits || [];
      let best = null;
      let bestD = radius;
      hits.forEach((ev) => {
        const d = Math.hypot((ev.x || 0) - mx, (ev.y || 0) - my);
        if (d < bestD) {
          bestD = d;
          best = ev;
        }
      });
      return best;
    }

    setActors(list) {
      if (!this._actorGroup) {
        this._actorGroup = new THREE.Group();
        this.scene.add(this._actorGroup);
      }
      while (this._actorGroup.children.length) {
        const c = this._actorGroup.children.pop();
        c.geometry?.dispose?.();
        if (c.material) {
          if (Array.isArray(c.material)) c.material.forEach((m) => m.dispose?.());
          else c.material.dispose?.();
        }
        this._actorGroup.remove(c);
      }
      (list || []).forEach((a) => {
        const r = Math.max(0.15, Number(a.r) || 0.3);
        const isPed = (a.kind || "").includes("ped");
        const mat = new THREE.MeshStandardMaterial({
          color: isPed ? 0xf59e0b : 0x38bdf8,
          roughness: 0.6,
          metalness: 0.15,
        });
        const h = isPed ? 1.5 : 1.0;
        const mesh = new THREE.Mesh(new THREE.CylinderGeometry(r * 0.7, r, h, 12), mat);
        mesh.position.set(Number(a.x) || 0, h / 2, -(Number(a.y) || 0));
        this._actorGroup.add(mesh);
      });
    }

    setObstacles(list) {
      if (!this._obsGroup) {
        this._obsGroup = new THREE.Group();
        this.scene.add(this._obsGroup);
      }
      while (this._obsGroup.children.length) {
        const c = this._obsGroup.children.pop();
        c.geometry?.dispose?.();
        c.material?.dispose?.();
        this._obsGroup.remove(c);
      }
      const mat = new THREE.MeshStandardMaterial({
        color: 0xdc2626,
        roughness: 0.7,
        metalness: 0.1,
        transparent: true,
        opacity: 0.85,
      });
      (list || []).forEach((o) => {
        const r = Math.max(0.15, Number(o.r) || 0.4);
        const mesh = new THREE.Mesh(new THREE.CylinderGeometry(r, r, 0.9, 18), mat.clone());
        mesh.position.set(Number(o.x) || 0, 0.45, -(Number(o.y) || 0));
        mesh.castShadow = true;
        this._obsGroup.add(mesh);
      });
    }

    _tickAngleRestore(dt) {
      // 仅跟随模式回弹；非跟随不恢复
      if (!this.follow) {
        this._angleDirty = false;
        this._restoring = false;
        return;
      }
      if (this.dragging) return;
      if (!this._angleDirty && !this._restoring) return;
      const now = performance.now();
      if (this._angleDirty && now - this._lastAngleAt >= ANGLE_IDLE_MS) {
        this._angleDirty = false;
        this._restoring = true;
      }
      if (!this._restoring) return;
      const t = Math.min(1, dt * RESTORE_SPEED);
      this.orbitYawOffset = lerpAngle(this.orbitYawOffset, this._homeYawOffset, t);
      this.orbitPitch += (this._homePitch - this.orbitPitch) * t;
      let diff = ((this.orbitYawOffset - this._homeYawOffset + Math.PI) % (Math.PI * 2)) - Math.PI;
      if (diff < -Math.PI) diff += Math.PI * 2;
      if (Math.abs(diff) < 0.01 && Math.abs(this.orbitPitch - this._homePitch) < 0.01) {
        this.orbitYawOffset = this._homeYawOffset;
        this.orbitPitch = this._homePitch;
        this._restoring = false;
      }
    }

    _updateCamera() {
      const { x, y, yaw } = this.pose;
      const focusMapX = x + (this.follow ? 0 : this.panX);
      const focusMapY = y + (this.follow ? 0 : this.panY);
      const cx = focusMapX;
      const cz = -focusMapY;

      // 车头方向 (map)：(cos yaw, sin yaw)
      // 正后方 = yaw+π；再加用户左右偏移
      const backAng = yaw + Math.PI + this.orbitYawOffset;
      const pitch = this.orbitPitch;
      const dist = this.camDist;
      const horiz = Math.cos(pitch) * dist;
      const mapOx = Math.cos(backAng) * horiz;
      const mapOy = Math.sin(backAng) * horiz;
      // map (x,y) → three (x, 0, -y)
      this.camera.position.set(cx + mapOx, Math.sin(pitch) * dist, cz - mapOy);
      this.camera.lookAt(cx, 0.42, cz);
    }

    _loop() {
      const now = performance.now();
      const dt = Math.min(0.05, (now - this._lastFrame) / 1000);
      this._lastFrame = now;
      this._tickAngleRestore(dt);
      this._updateCamera();
      this.renderer.render(this.scene, this.camera);
      this._raf = requestAnimationFrame(() => this._loop());
    }

    dispose() {
      if (this._raf) cancelAnimationFrame(this._raf);
    }
  }

  global.Sim3DView = Sim3DView;
})(window);
