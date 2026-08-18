/**
 * AGV Nav debug card renderers + registry builders.
 * Depends on: AgvComponentSystem, global state/api/view injected via install().
 */
(function (global) {
  function fmt(n, d) {
    d = d == null ? 2 : d;
    const v = Number(n);
    return Number.isFinite(v) ? v.toFixed(d) : "—";
  }
  function telem(d) {
    return (d && d.telemetry) || [];
  }
  function kv(rows) {
    return `<div class="bb-kv">${rows
      .map(([k, v]) => `<div class="k">${k}</div><div class="v">${v}</div>`)
      .join("")}</div>`;
  }
  function throttleSnap(card, ms, fn) {
    card._onSnap = (snap) => {
      if (card._uiFrozen) return;
      const t = performance.now();
      if (card._uiAt && t - card._uiAt < ms) return;
      card._uiAt = t;
      fn(snap);
    };
  }
  function drawSeries(canvas, series, keys, colors, h) {
    if (!canvas || !series || !series.length) return;
    const ctx = canvas.getContext("2d");
    const w = (canvas.width = canvas.clientWidth || 320);
    h = h || 140;
    canvas.height = h;
    ctx.fillStyle = "#020617";
    ctx.fillRect(0, 0, w, h);
    ctx.strokeStyle = "rgba(148,163,184,0.25)";
    ctx.beginPath();
    ctx.moveTo(0, h / 2);
    ctx.lineTo(w, h / 2);
    ctx.stroke();
    let min = Infinity,
      max = -Infinity;
    keys.forEach((k) =>
      series.forEach((r) => {
        const v = Number(r[k]);
        if (Number.isFinite(v)) {
          min = Math.min(min, v);
          max = Math.max(max, v);
        }
      })
    );
    if (!Number.isFinite(min)) return;
    if (max - min < 1e-6) {
      min -= 0.1;
      max += 0.1;
    }
    keys.forEach((key, ki) => {
      ctx.strokeStyle = colors[ki % colors.length];
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      let started = false;
      series.forEach((r, i) => {
        const v = Number(r[key]);
        if (!Number.isFinite(v)) return;
        const x = (i / Math.max(1, series.length - 1)) * (w - 4) + 2;
        const y = h - 4 - ((v - min) / (max - min)) * (h - 8);
        if (!started) {
          ctx.moveTo(x, y);
          started = true;
        } else ctx.lineTo(x, y);
      });
      ctx.stroke();
    });
  }

  function install(ctx) {
    const { state, api, view } = ctx;

    function chartCard(keys, colors, height, metaFn) {
      return (card) => {
        const body = card.querySelector(".widget-body");
        body.innerHTML = `<div class="bb-meta"></div><canvas class="bb-chart"></canvas>
          <div class="bb-legend">${keys
            .map((k, i) => `<span><i style="background:${colors[i]}"></i>${k}</span>`)
            .join("")}</div>`;
        const canvas = body.querySelector("canvas");
        throttleSnap(card, 200, (snap) => {
          const d = snap.debug || {};
          if (metaFn) body.querySelector(".bb-meta").innerHTML = metaFn(d, snap);
          drawSeries(canvas, telem(d), keys, colors, height || 150);
        });
        card._onResize = () => state.snap && card._onSnap?.(state.snap);
      };
    }

    const inits = {
      dbg_dock(card) {
        const body = card.querySelector(".widget-body");
        body.innerHTML = `
          <div class="row" style="gap:6px;flex-wrap:wrap;margin-bottom:6px">
            <select id="dbgLevel">
              <option value="BASIC">BASIC</option>
              <option value="ADVANCED" selected>ADVANCED</option>
              <option value="FULL">FULL</option>
              <option value="OFF">OFF</option>
            </select>
            <button class="btn" type="button" id="dbgFreeze">PAUSE</button>
            <button class="btn" type="button" id="dbgCapture">Capture</button>
            <button class="btn" type="button" id="dbgExport">Export</button>
          </div>
          <div class="tag" id="dbgSession">session —</div>
          <div class="dbg-layers" id="dbgLayers"></div>`;
        const layerBox = body.querySelector("#dbgLayers");
        [
          ["rawPath", "Raw"],
          ["processedPath", "Processed"],
          ["executedPath", "Executed"],
          ["actualTrace", "Actual"],
          ["mapEvents", "Events"],
          ["candidates", "Top-K"],
          ["footprints", "Foot"],
          ["sectors", "Radar"],
          ["lookahead", "LA"],
          ["futurePreview", "FutPrev"],
          ["collision", "Coll"],
        ].forEach(([k, lab]) => {
          const labEl = document.createElement("label");
          labEl.innerHTML = `<input type="checkbox" ${state.debugLayers[k] ? "checked" : ""}/> ${lab}`;
          layerBox.appendChild(labEl);
          labEl.querySelector("input").onchange = (e) => {
            state.debugLayers[k] = !!e.target.checked;
            if (state.snap) view.setDebugOverlay(state.snap.debug || {}, state.snap.agv || {}, state.debugLayers);
          };
        });
        const sel = body.querySelector("#dbgLevel");
        sel.value = state.debugLevel || "ADVANCED";
        sel.onchange = async (e) => {
          state.debugLevel = e.target.value;
          await api("/api/nav/debug/level", { method: "POST", body: JSON.stringify({ level: e.target.value }) });
        };
        body.querySelector("#dbgFreeze").onclick = async () => {
          state.debugFreeze = !state.debugFreeze;
          body.querySelector("#dbgFreeze").textContent = state.debugFreeze ? "RESUME" : "PAUSE";
          document.querySelectorAll(".widget-card.glass-comp").forEach((c) => {
            c._uiFrozen = state.debugFreeze;
          });
          await api("/api/nav/debug/freeze", { method: "POST", body: JSON.stringify({ freeze: state.debugFreeze }) });
        };
        body.querySelector("#dbgCapture").onclick = async () => {
          const r = await api("/api/nav/debug/capture", { method: "POST", body: "{}" });
          state.lastCapture = r.capture || r.debug;
          alert("Captured " + (state.lastCapture?.session_id || ""));
        };
        body.querySelector("#dbgExport").onclick = () => {
          const payload = state.lastCapture || (state.snap && state.snap.debug) || {};
          const blob = new Blob([JSON.stringify(payload, null, 2)], { type: "application/json" });
          const a = document.createElement("a");
          a.href = URL.createObjectURL(blob);
          a.download = `agv_nav_debug_${payload.session_id || Date.now()}.json`;
          a.click();
        };
        throttleSnap(card, 400, (snap) => {
          const d = snap.debug || {};
          body.querySelector("#dbgSession").textContent = `session ${d.session_id || "—"} · telem ${
            d.telem_ring_len ?? telem(d).length
          }`;
        });
      },
      dbg_motion(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 180, (snap) => {
          const m = snap.debug?.motion || {};
          const st = snap.debug?.status || {};
          const ms = m.motion_state || st.motion_state || "STOPPED";
          body.innerHTML =
            `<div class="bb-state ${ms}">${ms}</div>` +
            kv([
              ["vx", fmt(m.vx ?? st.vx, 3) + " m/s"],
              ["w", fmt(m.w ?? st.w, 3) + " rad/s"],
              ["phase", m.phase || st.phase || "—"],
              ["mode", m.control_mode || st.control_mode || "—"],
            ]);
        });
      },
      dbg_vel_chain(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 180, (snap) => {
          const vc = snap.debug?.velocity_chain || {};
          const note = vc.note || "OK";
          const cls = note.includes("CUT") || note.includes("STOP") ? "cut" : note.includes("DELAY") ? "warn" : "ok";
          body.innerHTML = `<div class="bb-chain">
            <div><span>MPPI</span><b>${fmt(vc.mppi_vx, 3)}</b></div>
            <div><span>CMD</span><b>${fmt(vc.cmd_vx, 3)}</b></div>
            <div><span>SAFETY</span><b>${fmt(vc.safe_vx, 3)}</b></div>
            <div><span>STATE</span><b>${fmt(vc.state_vx, 3)}</b></div>
          </div><div class="bb-note ${cls}">${note}</div>`;
        });
      },
      dbg_lon_v: chartCard(
        ["mppi_vx", "safe_vx", "state_vx"],
        ["#34d399", "#f472b6", "#fbbf24"],
        150,
        (d) => {
          const last = telem(d).slice(-1)[0] || {};
          return kv([["state vx", fmt(last.state_vx, 3) + " m/s"]]);
        }
      ),
      dbg_lon_a: chartCard(["ax"], ["#fbbf24"], 140, (d) => {
        const m = d.motion || {};
        return kv([
          ["ax", fmt(m.ax, 3) + " m/s²"],
          ["max", fmt(m.max_ax, 3)],
          ["min", fmt(m.min_ax, 3)],
        ]);
      }),
      dbg_ang: chartCard(["state_w", "mppi_w"], ["#60a5fa", "#34d399"], 130, (d) => {
        const st = d.steering || {};
        return kv([
          ["w", fmt(st.state_w, 3)],
          ["ratio", fmt((st.steering_execution_ratio || 0) * 100, 0) + "%"],
        ]);
      }),
      dbg_ang_a: chartCard(["alpha"], ["#f472b6"], 130, (d) => {
        const m = d.motion || {};
        return kv([
          ["α", fmt(m.alpha, 3)],
          ["max", fmt(m.max_alpha, 3)],
        ]);
      }),
      dbg_track(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 200, (snap) => {
          const st = snap.debug?.status || {};
          const ct = snap.debug?.controller || {};
          const last = telem(snap.debug).slice(-1)[0] || {};
          const div = snap.debug?.tracking_divergence?.first_divergence;
          body.innerHTML =
            kv([
              ["progress", fmt(st.path_progress_s, 3) + " m"],
              ["rate", fmt(last.path_progress_rate ?? ct.path_progress_rate, 3) + " m/s"],
              ["lateral", fmt(ct.lateral_err ?? last.lateral_error, 3) + " m"],
              ["heading", fmt(ct.heading_err ?? last.heading_error, 3) + " rad"],
              ["lat rate", fmt(last.lateral_error_rate, 3)],
            ]) +
            (div
              ? `<div class="bb-note warn">FIRST DIVERGENCE +${fmt(div.since_session_s, 2)}s</div>`
              : "");
        });
      },
      dbg_track_err: chartCard(
        ["lateral_error", "heading_error"],
        ["#60a5fa", "#f472b6"],
        150,
        () => `<div class="tag">lateral / heading</div>`
      ),
      dbg_clear(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 180, (snap) => {
          const wa = snap.debug?.wall_approach || {};
          const pvc = snap.debug?.path_vs_actual_clearance || {};
          const mism = snap.debug?.radar?.clearance_collision_mismatch;
          body.innerHTML =
            kv([
              ["front", fmt(snap.debug?.radar?.front_near, 2) + " m"],
              ["actual clr", fmt(pvc.actual_vehicle_clearance ?? wa.actual_clearance, 2) + " m"],
              ["path clr", fmt(pvc.planned_path_clearance ?? wa.path_clearance, 2) + " m"],
              ["clr rate", fmt(wa.clearance_rate, 3) + " m/s"],
              ["tag", wa.approach_tag || "—"],
            ]) + (mism ? `<div class="bb-note cut">CLEARANCE / COLLISION MISMATCH</div>` : "");
        });
      },
      dbg_clr_curve: chartCard(
        ["front_near", "actual_clearance", "path_clearance"],
        ["#ef4444", "#fbbf24", "#34d399"],
        150,
        () => `<div class="tag">front / actual / path</div>`
      ),
      dbg_path_prog: chartCard(
        ["path_progress_s", "path_progress_rate"],
        ["#34d399", "#60a5fa"],
        140,
        (d) => {
          const last = telem(d).slice(-1)[0] || {};
          return kv([
            ["progress", fmt(last.path_progress_s, 3) + " m"],
            ["rate", fmt(last.path_progress_rate, 3)],
          ]);
        }
      ),
      dbg_radar(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 250, (snap) => {
          const r = snap.debug?.radar || {};
          body.innerHTML = kv([
            ["front", fmt(r.front_near, 2) + " m"],
            ["rear", fmt(r.rear_near, 2) + " m"],
            ["metric", r.front_metric || "second_nearest"],
          ]);
        });
      },
      dbg_planner(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 250, (snap) => {
          const pl = snap.debug?.planner || {};
          const lp = snap.debug?.local_planner || {};
          body.innerHTML = kv([
            ["quality", pl.path_quality?.status || "—"],
            ["length", fmt(pl.final_path_length, 2) + " m"],
            ["ratio", fmt(pl.path_ratio, 2)],
            ["min clr", fmt(pl.min_clearance, 2) + " m"],
            ["LOS", pl.direct_path_safe ? "SAFE" : "BLOCKED"],
            ["forward", lp.forward_trajectory || "—"],
          ]);
        });
      },
      dbg_cand(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 250, (snap) => {
          const d = snap.debug || {};
          const lp = d.local_planner || {};
          const log = (d.candidate_switch_log || []).slice(-4).reverse();
          body.innerHTML =
            kv([
              ["selected", "#" + (lp.selected_candidate ?? "—")],
              ["switches", lp.candidate_switch_count ?? 0],
              ["best cost", fmt(lp.best_cost, 2)],
            ]) +
            log
              .map(
                (r) =>
                  `<div class="dbg-cand">#${r.previous_candidate}→#${r.new_candidate} vx ${fmt(r.previous_vx, 2)}→${fmt(
                    r.new_vx,
                    2
                  )} w ${fmt(r.previous_w, 2)}→${fmt(r.new_w, 2)}</div>`
              )
              .join("");
        });
      },
      dbg_safety(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 180, (snap) => {
          const sf = snap.debug?.safety || snap.safety || {};
          const sev = sf.nav_ui_severity || snap.nav?.nav_ui_severity || "NORMAL";
          const sevCls = sev === "FAILED" ? "cut" : sev === "RECOVERY" ? "warn" : sev === "DEGRADED" ? "warn" : "ok";
          body.innerHTML =
            `<div class="bb-state ${sevCls}">${sev}</div>` +
            kv([
              ["safe_vx_reason", sf.safe_vx_reason || snap.nav?.safe_vx_reason || "—"],
              ["planner", sf.planner_state || snap.nav?.planner_state || "—"],
              ["recovery", sf.recovery_state || snap.nav?.recovery_state || "—"],
              ["fp clr", fmt(sf.footprint_clearance_m ?? snap.nav?.footprint_clearance_m, 3)],
              ["pred min", fmt(sf.predicted_min_clearance_m ?? snap.nav?.predicted_min_clearance_m, 3)],
              ["decision", sf.decision || "—"],
              ["block", sf.block_reason || "—"],
              ["front", fmt(sf.front_near, 2)],
              ["rear", fmt(sf.rear_near, 2)],
              ["collision", sf.collision ? "TRUE" : "false"],
            ]);
        });
      },
      dbg_recovery(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 200, (snap) => {
          const rc = snap.debug?.recovery || {};
          const st = snap.debug?.status || {};
          const nav = snap.nav || {};
          const atts = snap.debug?.recovery_attempts || [];
          const last = atts[atts.length - 1];
          const pstate = nav.planner_state || rc.planner_state || "—";
          const isFailed = pstate === "NAVIGATION_FAILED" || nav.stop_reason === "NAVIGATION_FAILED";
          const isRec = pstate === "LOCAL_RECOVERY" || nav.recovery_state !== "NONE";
          const badge = isFailed ? "FAILED" : isRec ? "RECOVERY" : pstate;
          const badgeCls = isFailed ? "cut" : isRec ? "warn" : "ok";
          body.innerHTML =
            `<div class="bb-state ${badgeCls}">${badge}</div>` +
            kv([
              ["planner", pstate],
              ["failure", nav.planner_failure_reason || rc.planner_failure_reason || "—"],
              ["phase", rc.phase || st.phase || nav.phase || "—"],
              ["action", nav.recovery_state || rc.last_recovery_action || "—"],
              ["attempt", `${nav.recovery_attempt ?? rc.recovery_attempts ?? st.recovery_attempts ?? 0} / ${rc.max_attempts ?? 3}`],
              ["last", rc.last_recovery_action || "—"],
              ["result", last?.result || "—"],
            ]) +
            (last?.result === "WORSE" ? `<div class="bb-note cut">RECOVERY_MADE_GEOMETRY_WORSE</div>` : "");
        });
      },
      dbg_timeline(card) {
        const body = card.querySelector(".widget-body");
        body.style.display = "flex";
        body.style.flexDirection = "column";
        body.innerHTML = `<div class="bb-timeline" id="tl" style="flex:1;min-height:0;max-height:none;height:100%"></div>`;
        throttleSnap(card, 300, (snap) => {
          const ev = (snap.debug?.events || []).slice().reverse().slice(0, 80);
          body.querySelector("#tl").innerHTML =
            ev
              .map((e) => {
                const t = new Date((e.ts || 0) * 1000);
                const ts = `${String(t.getHours()).padStart(2, "0")}:${String(t.getMinutes()).padStart(2, "0")}:${String(
                  t.getSeconds()
                ).padStart(2, "0")}.${String(t.getMilliseconds()).padStart(3, "0")}`;
                const cat = e.category || e.type || "INFO";
                return `<div class="bb-tl-row"><span>${ts}</span><span class="bb-tl-cat ${cat}">${cat}</span><span>${
                  e.event || ""
                } ${e.message && e.message !== e.event ? e.message : ""}</span></div>`;
              })
              .join("") || '<div class="tag">no events</div>';
        });
      },
      dbg_incident(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 400, (snap) => {
          const inc = snap.debug?.incident;
          if (!inc) {
            body.innerHTML = '<div class="tag">waiting for incident window…</div>';
            return;
          }
          const kfs = inc.keyframes || [];
          body.innerHTML =
            `<div class="bb-note cut">${inc.trigger || "INCIDENT"}${inc.live ? " (live)" : ""}</div>` +
            kv([
              ["lat Δ", fmt(inc.summary?.lat_delta, 3)],
              ["clr Δ", fmt(inc.summary?.clr_delta, 3)],
              ["progress", `${fmt(inc.summary?.progress_start, 2)}→${fmt(inc.summary?.progress_end, 2)}`],
            ]) +
            `<div class="bb-timeline" style="max-height:160px">${kfs
              .map(
                (k) =>
                  `<div class="bb-tl-row"><span>${k.label}</span><span>vx ${fmt(k.vx, 2)}</span><span>lat ${fmt(
                    k.lateral_error,
                    2
                  )} clr ${fmt(k.actual_clearance ?? k.front_near, 2)} ${k.phase || ""}</span></div>`
              )
              .join("")}</div>`;
        });
      },
      dbg_exec(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 250, (snap) => {
          const ex = snap.debug?.execution_state || {};
          const an = snap.debug?.anomalies || {};
          body.innerHTML =
            kv([
              ["state", ex.current || "—"],
              ["in state", fmt(ex.time_in_state_s, 2) + " s"],
            ]) +
            (an.reverse_with_forward_available
              ? `<div class="bb-note cut">REVERSE_WITH_FORWARD_AVAILABLE</div>`
              : "");
        });
      },
      dbg_why(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 200, (snap) => {
          const diag = snap.debug?.diagnostics || {};
          const st = snap.debug?.status || {};
          body.innerHTML = `
            <div class="bb-note ${diag.primary_reason && diag.primary_reason !== "NONE" ? "cut" : "ok"}">
              ${diag.why_stopped || diag.primary_reason || "NONE"}
            </div>
            <div style="margin-top:6px;line-height:1.4">${diag.explanation || "—"}</div>
            <div class="tag" style="margin-top:6px">stop_reason: ${st.stop_reason || "NONE"}</div>
            ${(diag.anomalies || []).map((a) => `<div class="bb-note warn">${a}</div>`).join("")}`;
        });
      },
      dbg_summary(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 400, (snap) => {
          const s = snap.debug?.run_summary;
          if (!s) {
            body.innerHTML = '<div class="tag">waiting for session end…</div>';
            return;
          }
          body.innerHTML = kv([
            ["id", s.session_id || "—"],
            ["duration", fmt(s.duration_s, 1) + " s"],
            ["fwd/rev", `${fmt(s.forward_distance_m, 2)} / ${fmt(s.reverse_distance_m, 2)} m`],
            ["progress", fmt(s.path_progress_m, 2) + " m"],
            ["min act clr", fmt(s.min_actual_clearance, 2)],
            ["min path clr", fmt(s.min_path_clearance, 2)],
            ["switches", s.candidate_switches ?? "—"],
            ["recovery", s.recovery_attempts ?? "—"],
            ["final", s.final || "—"],
            ["primary", s.primary_diagnosis || "—"],
            ["secondary", (s.secondary_diagnosis || []).join(", ") || "—"],
          ]);
        });
      },
      dbg_post(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 400, (snap) => {
          const pm = snap.debug?.post_mortem;
          if (!pm) {
            body.innerHTML = '<div class="tag">no post-mortem yet</div>';
            return;
          }
          const fa = pm.first_anomaly || {};
          body.innerHTML = `
            <div class="bb-note cut">FIRST ANOMALY</div>
            <div class="tag">${fa.ts ? new Date(fa.ts * 1000).toLocaleTimeString() : "—"} · ${fa.event || "—"}</div>
            <div style="margin-top:6px">${fa.message || ""}</div>`;
        });
      },
      dbg_rev(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 300, (snap) => {
          const dec = (snap.debug?.reverse_decisions || []).slice(-1)[0];
          const fr = snap.debug?.forward_reverse || {};
          if (!dec && !fr.selected_maneuver) {
            body.innerHTML = '<div class="tag">no reverse decision yet</div>';
            return;
          }
          body.innerHTML =
            kv([
              ["reason", (dec && dec.reason) || fr.selected_maneuver || "—"],
              ["fwd feasible", (fr.forward_feasible ?? dec?.forward_feasible) ? "TRUE" : "false"],
              ["fwd reason", fr.forward_reason || "—"],
              ["best fwd vx", fmt(fr.best_forward_vx ?? dec?.best_forward_vx, 2)],
              ["best rev vx", fmt(fr.best_reverse_vx ?? dec?.best_reverse_vx, 2)],
              ["front", fmt(dec?.front, 2)],
            ]) +
            (dec?.anomaly ? `<div class="bb-note cut">${dec.anomaly}</div>` : "");
        });
      },
      dbg_maneuver(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 200, (snap) => {
          const m = snap.debug?.maneuver || {};
          const cap = m.capture || {};
          body.innerHTML = kv([
            ["maneuver", m.mode || "—"],
            ["reason", m.reason || "—"],
            ["target θ", fmt(m.target_heading, 2)],
            ["heading err", fmt(m.heading_error, 2)],
            ["capture dist", fmt(cap.distance, 2)],
            ["rot safe", m.rotation_safe ? "YES" : "no"],
            ["turn ok", m.turn_feasible ? "YES" : "no"],
            ["L/R free", `${fmt(m.left_free, 1)} / ${fmt(m.right_free, 1)}`],
          ]);
        });
      },
      dbg_fwd_rev(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 200, (snap) => {
          const fr = snap.debug?.forward_reverse || {};
          const lp = snap.debug?.local_planner || {};
          body.innerHTML = kv([
            ["forward", fr.forward_feasible ? "FEASIBLE" : "NO"],
            ["fwd reason", fr.forward_reason || lp.forward_reason || "—"],
            ["best fwd vx", fmt(fr.best_forward_vx, 2)],
            ["best fwd cost", fmt(fr.best_forward_cost, 1)],
            ["reverse", fr.reverse_feasible ? "FEASIBLE" : "no"],
            ["best rev vx", fmt(fr.best_reverse_vx, 2)],
            ["best rev cost", fmt(fr.best_reverse_cost, 1)],
            ["selected", fr.selected_maneuver || "—"],
          ]);
        });
      },
      dbg_capture(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 200, (snap) => {
          const cap = snap.debug?.path_capture || snap.debug?.maneuver?.capture || {};
          const pt = cap.point || {};
          body.innerHTML = kv([
            ["capture", cap.available ? "AVAILABLE" : "NONE"],
            ["distance", fmt(cap.distance, 2)],
            ["heading err", fmt(cap.heading_error, 2)],
            ["clearance", fmt(cap.clearance, 2)],
            ["point x", fmt(pt.x, 2)],
            ["point y", fmt(pt.y, 2)],
          ]);
        });
      },
      dbg_maneuver_tl(card) {
        const body = card.querySelector(".widget-body");
        body.innerHTML = `<div class="bb-meta"></div><canvas class="bb-chart"></canvas>
          <div class="bb-legend">
            <span><i style="background:#34d399"></i>FWD</span>
            <span><i style="background:#60a5fa"></i>TURN</span>
            <span><i style="background:#fbbf24"></i>ALIGN</span>
            <span><i style="background:#a78bfa"></i>REPOS</span>
            <span><i style="background:#f87171"></i>REV</span>
            <span><i style="background:#94a3b8"></i>STOP</span>
          </div>`;
        const canvas = body.querySelector("canvas");
        const colors = {
          FORWARD_TRACK: "#34d399",
          FORWARD_TURN: "#60a5fa",
          ALIGN: "#fbbf24",
          TURN_IN_PLACE: "#f59e0b",
          REPOSITION: "#a78bfa",
          REVERSE_ESCAPE: "#f87171",
          POST_TURN: "#2dd4bf",
          REPLAN: "#fb923c",
          SAFE_STOP: "#94a3b8",
          WAIT_FOR_CLEARANCE: "#64748b",
          IDLE: "#475569",
        };
        throttleSnap(card, 250, (snap) => {
          const series = telem(snap.debug || {});
          const meta = body.querySelector(".bb-meta");
          const cur = snap.debug?.maneuver?.mode || "—";
          meta.innerHTML = `now: <b>${cur}</b>`;
          if (!canvas || !series.length) return;
          const ctx = canvas.getContext("2d");
          const w = (canvas.width = canvas.clientWidth || 320);
          const h = (canvas.height = 56);
          ctx.fillStyle = "#020617";
          ctx.fillRect(0, 0, w, h);
          const n = series.length;
          for (let i = 0; i < n; i++) {
            const m = series[i].maneuver_mode || "IDLE";
            ctx.fillStyle = colors[m] || "#334155";
            const x0 = (i / n) * w;
            const x1 = ((i + 1) / n) * w;
            ctx.fillRect(x0, 8, Math.max(1, x1 - x0), h - 16);
          }
        });
        card._onResize = () => state.snap && card._onSnap?.(state.snap);
      },
      dbg_local_man(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 200, (snap) => {
          const lm = snap.debug?.local_maneuver || {};
          const f = lm.forward || {};
          const L = lm.left || {};
          const R = lm.right || {};
          body.innerHTML = kv([
            ["current", lm.decision || "—"],
            ["reason", lm.reason || "—"],
            ["forward", f.quality || (f.feasible ? "OK" : "BLOCKED")],
            ["left", L.quality || (L.feasible ? "OK" : "BLOCKED")],
            ["right", R.quality || (R.feasible ? "OK" : "BLOCKED")],
            ["selected", lm.decision || "—"],
            ["L clr / cap", `${fmt(L.clr, 2)} / ${fmt(L.capture, 2)}`],
            ["R clr / cap", `${fmt(R.clr, 2)} / ${fmt(R.capture, 2)}`],
          ]);
        });
      },
      dbg_nav_policy(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 200, (snap) => {
          const p = snap.debug?.nav_policy || {};
          const c = p.corridor || {};
          const pr = p.profile || {};
          body.innerHTML = kv([
            ["state", p.state || "—"],
            ["behavior", p.behavior || "—"],
            ["reason", p.reason || "—"],
            ["scene", p.scene || "—"],
            ["corridor", `${fmt(c.half_width, 2)}m (${c.mode || "—"})`],
            ["deviation", `${fmt(c.lateral_error ?? p.local_deviation, 2)} / ${fmt(p.max_deviation_m, 2)}`],
            ["path_follow_w", fmt(p.path_follow_weight, 2)],
            ["profile", pr.name || "—"],
            ["compare/replan/rev", `${p.allow_side_compare ? "Y" : "n"} / ${p.allow_replan ? "Y" : "n"} / ${p.allow_recovery ? "Y" : "n"}`],
          ]);
        });
      },
      dbg_side_chain(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 200, (snap) => {
          const op = snap.debug?.obstacle_preview || {};
          const nav = snap.nav || {};
          const ec = snap.debug?.nav_policy?.execution_corridor || op.execution_corridor || nav.execution_corridor || {};
          const probe = op.probe_side || op.side_probe?.preferred_side || "—";
          const commit = op.commit_side || op.committed_side || snap.debug?.avoidance_phase?.committed_side || "—";
          const execSide = op.execution_side || ec.committed_side || ec.mode || "—";
          const inc = op.side_inconsistency || "";
          const w = nav.mppi_w ?? snap.debug?.velocity_chain?.requested_omega;
          const omegaSign = w == null ? "—" : (w > 0.02 ? "LEFT (+ω)" : w < -0.02 ? "RIGHT (-ω)" : "STRAIGHT");
          body.innerHTML = kv([
            ["PROBE", probe],
            ["COMMIT", commit],
            ["EXECUTION", execSide],
            ["corridor.active", ec.active != null ? String(ec.active) : "—"],
            ["corridor.mode", ec.mode || "—"],
            ["inconsistency", inc || "—"],
            ["ω sign", omegaSign],
            ["L/R conf", `${fmt(op.probe_confidence_left ?? op.side_probe?.left_confidence, 2)} / ${fmt(op.probe_confidence_right ?? op.side_probe?.right_confidence, 2)}`],
            ["L/R valid", `${op.left_valid ?? op.side_probe?.left_valid ?? "—"} / ${op.right_valid ?? op.side_probe?.right_valid ?? "—"}`],
          ]);
        });
      },
      dbg_phase4_probe(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 250, (snap) => {
          const pr = snap.debug?.probe || snap.debug?.phase4?.probe || {};
          const f = pr.forward || {};
          const b = pr.backward || {};
          const L = pr.left || {};
          const R = pr.right || {};
          const t = pr.turn_in_place || {};
          const fmt = (r) => {
            if (!r || !r.status) return "—";
            const clr = r.min_clearance != null ? Number(r.min_clearance).toFixed(2) : "—";
            const extra = r.soft_risk ? " soft" : "";
            const reason = r.failure_reason && r.failure_reason !== "NONE" ? ` ${r.failure_reason}` : "";
            return `${r.status} clr=${clr}${extra}${reason}`;
          };
          body.innerHTML = kv([
            ["PROBE", pr.implemented ? String(pr.status || "OK") : "NOT_IMPLEMENTED"],
            ["bundle_ms", pr.bundle_ms != null ? Number(pr.bundle_ms).toFixed(2) : "—"],
            ["FORWARD", fmt(f)],
            ["  stop_mgn", f.stopping_margin != null ? Number(f.stopping_margin).toFixed(2) : "—"],
            ["BACKWARD", fmt(b)],
            ["  escape", b.escape_available == null ? "—" : String(b.escape_available)],
            ["LEFT", fmt(L)],
            ["RIGHT", fmt(R)],
            ["TURN", fmt(t)],
            ["  max_yaw", t.max_safe_yaw_delta != null ? (Number(t.max_safe_yaw_delta) * 57.3).toFixed(0) + "°" : "—"],
            ["note", "evidence-only; no auto switch"],
          ]);
        });
      },
      dbg_phase4_commit(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 250, (snap) => {
          const c = snap.debug?.commitment || snap.debug?.phase4?.commitment || {};
          const sides = snap.debug?.sides || snap.debug?.phase4?.sides || {};
          const sw = snap.debug?.side_switch || snap.debug?.phase4?.side_switch || {};
          body.innerHTML = kv([
            ["COMMITMENT", c.implemented ? (c.active ? "LOCKED" : "READY") : "NOT IMPLEMENTED"],
            ["active", String(!!c.active)],
            ["commit.side", c.side || "NONE"],
            ["phase", c.phase || "NONE"],
            ["fail", c.failure_reason || "NONE"],
            ["hard_fail", String(!!c.hard_fail)],
            ["auth", sw.authorization_status || c.authorization_status || "—"],
            ["soft_streak", String(c.soft_fail_streak ?? 0)],
            ["age_s", c.age_s != null ? Number(c.age_s).toFixed(2) : "—"],
            ["— SIDES —", ""],
            ["policy avoid", sides.policy_avoid_side || "—"],
            ["commit", sides.commitment_side || "—"],
            ["selector", sides.selector_side || "—"],
            ["fsm", sides.fsm_side || "—"],
            ["controller", sides.controller_side || "—"],
            ["actual w", sides.actual_motion_side || "—"],
          ]);
        });
      },
      dbg_phase4_switch(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 250, (snap) => {
          const s = snap.debug?.side_switch || snap.debug?.phase4?.side_switch || {};
          const own = snap.debug?.ownership || snap.debug?.phase4?.ownership || {};
          const pr = snap.debug?.probe || snap.debug?.phase4?.probe || {};
          const gates = s.gates || {};
          const failed = (s.failed_gates || []).join(",") || "—";
          const gLine = (k) => (gates[k] == null ? "—" : gates[k] ? "PASS" : "FAIL");
          body.innerHTML = kv([
            ["AUTHORIZATION", s.authorized ? "AUTHORIZED" : s.status || s.authorization_status || "—"],
            ["reason", s.reason || s.primary_reason || "—"],
            ["from → to", `${s.from_side || s.current_side || "—"} → ${s.to_side || s.candidate_side || "—"}`],
            ["raw candidate", s.candidate_side || "—"],
            ["cur probe", s.current_probe_status || (pr.left && pr.left.status) || "—"],
            ["alt probe", s.alternative_probe_status || s.candidate_probe || "—"],
            ["turn", s.turn_probe_status || (pr.turn_in_place && pr.turn_in_place.status) || "—"],
            ["safety", gLine("safety_ok")],
            ["dynamic", gLine("dynamic_ok")],
            ["cooldown", gLine("cooldown_ok")],
            ["oscillation", gLine("oscillation_ok")],
            ["failed gates", failed],
            ["auth_ms", s.authorization_ms != null ? Number(s.authorization_ms).toFixed(3) : "—"],
            ["authority", own.side_switch_authority || own.side_switch_authority_current || "NavigationPolicy"],
          ]);
        });
      },
      dbg_phase4_traj(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 250, (snap) => {
          const pt = snap.debug?.physical_trajectory || snap.debug?.phase4?.physical_trajectory || {};
          const a = pt.active || {};
          const rec = snap.debug?.recovery || snap.debug?.phase4?.recovery || {};
          const bc = snap.debug?.breadcrumb || snap.debug?.phase4?.breadcrumb || {};
          body.innerHTML = kv([
            ["ACTIVE", a.source || "—"],
            ["kind", a.trajectory_kind || snap.nav?.physical_trajectory?.trajectory_kind || "—"],
            ["source", a.trajectory_source || a.source || "—"],
            ["control", String(a.control_eligible ?? snap.nav?.physical_trajectory?.control_eligible ?? "—")],
            ["viz_only", String(a.visualization_only ?? snap.nav?.physical_trajectory?.visualization_only ?? "—")],
            ["status", a.status || "—"],
            ["valid", a.valid == null ? "—" : String(!!a.valid)],
            ["length_m", a.length_m != null ? Number(a.length_m).toFixed(2) : "—"],
            ["duration_s", a.duration_s != null ? Number(a.duration_s).toFixed(2) : "—"],
            ["half_w", a.half_width_m != null ? Number(a.half_width_m).toFixed(3) : "—"],
            ["min_clr", a.min_clearance != null ? Number(a.min_clearance).toFixed(2) : "—"],
            ["collision", a.collision == null ? "—" : String(!!a.collision)],
            ["risk", a.risk || "—"],
            ["scene_id", a.scene_id ?? snap.nav?.scene_id ?? "—"],
            ["cycle_id", a.planner_cycle_id ?? snap.nav?.planner_cycle_id ?? "—"],
            ["— RETREAT / HISTORY —", ""],
            ["kind", (snap.nav?.retreat_trajectory || {}).trajectory_kind || rec.action || "—"],
            ["control", String((snap.nav?.retreat_trajectory || {}).control_eligible ?? "—")],
            ["— RECOVERY —", ""],
            ["class", rec.classification || "—"],
            ["action", rec.action || "—"],
            ["reason", rec.reason || "—"],
            ["allow_rev", rec.allowed == null ? "—" : String(!!rec.allowed)],
            ["— BREADCRUMB —", ""],
            ["points", String(bc.count ?? (bc.points || []).length ?? 0)],
            ["length_m", bc.length_m != null ? Number(bc.length_m).toFixed(2) : "—"],
          ]);
        });
      },
      dbg_policy_tl(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 300, (snap) => {
          const hist = (snap.debug?.nav_policy?.history || []).slice(-10).reverse();
          if (!hist.length) {
            body.innerHTML = '<div class="tag">no policy history</div>';
            return;
          }
          body.innerHTML = hist
            .map(
              (h) =>
                `<div class="bb-kv"><div class="k">${h.state || "?"}</div><div class="v">${h.reason || ""}</div></div>`
            )
            .join("");
        });
      },
      dbg_policy_w: chartCard(
        ["path_follow_weight", "local_deviation"],
        ["#38bdf8", "#fbbf24"],
        140,
        (d) => {
          const p = d.nav_policy || {};
          return ` beh <b>${p.behavior || "—"}</b>`;
        }
      ),
      dbg_man_cmp(card) {
        const body = card.querySelector(".widget-body");
        throttleSnap(card, 250, (snap) => {
          const rows = (snap.debug?.local_maneuver?.rows || []).slice(0, 8);
          if (!rows.length) {
            body.innerHTML = '<div class="tag">no comparison yet</div>';
            return;
          }
          const head = `<div class="bb-kv" style="grid-template-columns:70px 50px 50px 45px 55px 55px">
            <div class="k">Act</div><div class="k">OK</div><div class="k">Cost</div><div class="k">Clr</div><div class="k">Cap</div><div class="k">Prog</div>`;
          const mid = rows
            .map(
              (r) =>
                `<div class="v">${r.action}${r.selected ? " *" : ""}</div><div class="v">${r.feasible ? "Y" : "n"}</div><div class="v">${fmt(r.cost, 0)}</div><div class="v">${fmt(r.clr, 2)}</div><div class="v">${fmt(r.capture, 1)}</div><div class="v">${fmt(r.progress, 2)}</div>`
            )
            .join("");
          body.innerHTML = head + mid + "</div>";
        });
      },
      dbg_man_cost: chartCard(
        ["fwd_cost", "left_cost", "right_cost"],
        ["#94a3b8", "#34d399", "#60a5fa"],
        140,
        (d) => {
          const lm = d.local_maneuver || {};
          return `sel <b>${lm.decision || "—"}</b>`;
        }
      ),
    };

    function buildRegistry(coreInits) {
      const R = {
        nav_mission: {
          title: "导航",
          w: 520,
          h: 420,
          minW: 360,
          minH: 280,
          x: 16,
          y: 64,
          group: "CORE",
          init: coreInits.nav_mission,
        },
        agv_status: {
          title: "AGV 状态",
          w: 240,
          h: 220,
          minW: 180,
          minH: 140,
          x: 1100,
          y: 64,
          group: "CORE",
          init: coreInits.agv_status,
        },
        local_plans: {
          title: "局部候选",
          w: 760,
          h: 200,
          minW: 320,
          minH: 140,
          x: 16,
          y: 500,
          group: "PLANNING",
          init: coreInits.local_plans,
        },
        obstacles: {
          title: "障碍物",
          w: 300,
          h: 360,
          minW: 220,
          minH: 180,
          x: 760,
          y: 64,
          group: "SAFETY",
          init: coreInits.obstacles,
        },
        dbg_dock: { title: "DEBUG DOCK", w: 300, h: 220, minW: 220, minH: 140, x: 12, y: 58, group: "DEBUG", init: inits.dbg_dock },
        dbg_motion: { title: "MOTION", w: 220, h: 160, minW: 160, minH: 120, x: 12, y: 290, group: "CORE", init: inits.dbg_motion },
        dbg_vel_chain: {
          title: "VELOCITY CHAIN",
          w: 260,
          h: 200,
          minW: 180,
          minH: 140,
          x: 240,
          y: 290,
          group: "CONTROL",
          init: inits.dbg_vel_chain,
        },
        dbg_lon_v: {
          title: "LONGITUDINAL VELOCITY",
          w: 360,
          h: 220,
          minW: 260,
          minH: 160,
          x: 440,
          y: 520,
          group: "CONTROL",
          init: inits.dbg_lon_v,
        },
        dbg_lon_a: {
          title: "LONGITUDINAL ACCEL",
          w: 360,
          h: 220,
          minW: 260,
          minH: 160,
          x: 810,
          y: 520,
          group: "CONTROL",
          init: inits.dbg_lon_a,
        },
        dbg_ang: {
          title: "ANGULAR VELOCITY",
          w: 360,
          h: 220,
          minW: 260,
          minH: 160,
          x: 440,
          y: 290,
          group: "CONTROL",
          init: inits.dbg_ang,
        },
        dbg_ang_a: {
          title: "ANGULAR ACCEL",
          w: 360,
          h: 220,
          minW: 260,
          minH: 160,
          x: 810,
          y: 290,
          group: "CONTROL",
          init: inits.dbg_ang_a,
        },
        dbg_track: {
          title: "PATH TRACKING",
          w: 240,
          h: 220,
          minW: 180,
          minH: 140,
          x: 1000,
          y: 300,
          group: "PLANNING",
          init: inits.dbg_track,
        },
        dbg_track_err: {
          title: "TRACKING ERROR",
          w: 360,
          h: 220,
          minW: 260,
          minH: 160,
          x: 1100,
          y: 520,
          group: "CONTROL",
          init: inits.dbg_track_err,
        },
        dbg_clear: {
          title: "CLEARANCE",
          w: 260,
          h: 200,
          minW: 180,
          minH: 140,
          x: 12,
          y: 480,
          group: "SAFETY",
          init: inits.dbg_clear,
        },
        dbg_clr_curve: {
          title: "CLEARANCE CURVES",
          w: 360,
          h: 220,
          minW: 260,
          minH: 160,
          x: 280,
          y: 520,
          group: "SAFETY",
          init: inits.dbg_clr_curve,
        },
        dbg_path_prog: {
          title: "PATH PROGRESS",
          w: 360,
          h: 220,
          minW: 260,
          minH: 160,
          x: 520,
          y: 760,
          group: "CONTROL",
          init: inits.dbg_path_prog,
        },
        dbg_radar: { title: "RADAR", w: 200, h: 170, minW: 160, minH: 120, x: 1000, y: 120, group: "SAFETY", init: inits.dbg_radar },
        dbg_planner: {
          title: "PLANNER",
          w: 300,
          h: 220,
          minW: 200,
          minH: 140,
          x: 12,
          y: 120,
          group: "PLANNING",
          init: inits.dbg_planner,
        },
        dbg_cand: {
          title: "CANDIDATE",
          w: 340,
          h: 260,
          minW: 240,
          minH: 160,
          x: 1000,
          y: 540,
          group: "PLANNING",
          init: inits.dbg_cand,
        },
        dbg_safety: {
          title: "SAFETY",
          w: 240,
          h: 180,
          minW: 180,
          minH: 120,
          x: 320,
          y: 120,
          group: "SAFETY",
          init: inits.dbg_safety,
        },
        dbg_recovery: {
          title: "RECOVERY",
          w: 260,
          h: 200,
          minW: 180,
          minH: 140,
          x: 560,
          y: 120,
          group: "RECOVERY",
          init: inits.dbg_recovery,
        },
        dbg_timeline: {
          title: "EXECUTION TIMELINE",
          w: 520,
          h: 280,
          minW: 320,
          minH: 180,
          x: 710,
          y: 58,
          group: "RECOVERY",
          scrollBody: true,
          init: inits.dbg_timeline,
        },
        dbg_incident: {
          title: "DRIVE REPLAY",
          w: 420,
          h: 280,
          minW: 280,
          minH: 180,
          x: 710,
          y: 360,
          group: "DEBUG",
          scrollBody: true,
          init: inits.dbg_incident,
        },
        dbg_exec: {
          title: "EXECUTION STATE",
          w: 230,
          h: 180,
          minW: 160,
          minH: 120,
          x: 1240,
          y: 58,
          group: "RECOVERY",
          init: inits.dbg_exec,
        },
        dbg_why: {
          title: "WHY STOPPED",
          w: 280,
          h: 210,
          minW: 200,
          minH: 140,
          x: 840,
          y: 360,
          group: "DEBUG",
          init: inits.dbg_why,
        },
        dbg_summary: {
          title: "RUN SUMMARY",
          w: 280,
          h: 280,
          minW: 200,
          minH: 160,
          x: 1100,
          y: 760,
          group: "DEBUG",
          init: inits.dbg_summary,
        },
        dbg_post: {
          title: "POST-MORTEM",
          w: 320,
          h: 240,
          minW: 220,
          minH: 140,
          x: 760,
          y: 760,
          group: "DEBUG",
          init: inits.dbg_post,
        },
        dbg_rev: {
          title: "FORWARD vs REVERSE",
          w: 280,
          h: 220,
          minW: 200,
          minH: 140,
          x: 520,
          y: 360,
          group: "RECOVERY",
          init: inits.dbg_rev,
        },
        dbg_maneuver: {
          title: "MANEUVER",
          w: 260,
          h: 220,
          minW: 180,
          minH: 140,
          x: 320,
          y: 58,
          group: "MANEUVER",
          init: inits.dbg_maneuver,
        },
        dbg_fwd_rev: {
          title: "FORWARD / REVERSE",
          w: 260,
          h: 220,
          minW: 180,
          minH: 140,
          x: 590,
          y: 58,
          group: "MANEUVER",
          init: inits.dbg_fwd_rev,
        },
        dbg_capture: {
          title: "PATH CAPTURE",
          w: 240,
          h: 200,
          minW: 180,
          minH: 140,
          x: 860,
          y: 58,
          group: "MANEUVER",
          init: inits.dbg_capture,
        },
        dbg_maneuver_tl: {
          title: "MANEUVER TIMELINE",
          w: 420,
          h: 140,
          minW: 280,
          minH: 110,
          x: 320,
          y: 290,
          group: "MANEUVER",
          init: inits.dbg_maneuver_tl,
        },
        dbg_local_man: {
          title: "LOCAL MANEUVER",
          w: 300,
          h: 220,
          minW: 200,
          minH: 140,
          x: 12,
          y: 460,
          group: "MANEUVER",
          init: inits.dbg_local_man,
        },
        dbg_man_cmp: {
          title: "MANEUVER COMPARISON",
          w: 400,
          h: 220,
          minW: 280,
          minH: 140,
          x: 320,
          y: 460,
          group: "MANEUVER",
          init: inits.dbg_man_cmp,
        },
        dbg_man_cost: {
          title: "MANEUVER COST",
          w: 360,
          h: 200,
          minW: 260,
          minH: 140,
          x: 740,
          y: 460,
          group: "MANEUVER",
          init: inits.dbg_man_cost,
        },
        dbg_nav_policy: {
          title: "NAVIGATION POLICY",
          w: 320,
          h: 260,
          minW: 220,
          minH: 160,
          x: 12,
          y: 700,
          group: "POLICY",
          init: inits.dbg_nav_policy,
        },
        dbg_side_chain: {
          title: "SIDE PROBE / COMMIT / EXEC",
          w: 300,
          h: 220,
          minW: 220,
          minH: 160,
          x: 12,
          y: 1260,
          group: "SAFETY",
          init: inits.dbg_side_chain,
        },
        dbg_phase4_probe: {
          title: "NAV PROBE",
          w: 280,
          h: 220,
          minW: 200,
          minH: 140,
          x: 12,
          y: 980,
          group: "PHASE4",
          init: inits.dbg_phase4_probe,
        },
        dbg_phase4_commit: {
          title: "AVOIDANCE COMMIT",
          w: 300,
          h: 260,
          minW: 220,
          minH: 160,
          x: 300,
          y: 980,
          group: "PHASE4",
          init: inits.dbg_phase4_commit,
        },
        dbg_phase4_switch: {
          title: "SIDE SWITCH",
          w: 340,
          h: 280,
          minW: 240,
          minH: 160,
          x: 620,
          y: 980,
          group: "PHASE4",
          init: inits.dbg_phase4_switch,
        },
        dbg_phase4_traj: {
          title: "PHYSICAL TRAJECTORY",
          w: 320,
          h: 300,
          minW: 220,
          minH: 160,
          x: 980,
          y: 980,
          group: "PHASE4",
          init: inits.dbg_phase4_traj,
        },
        dbg_policy_tl: {
          title: "POLICY TIMELINE",
          w: 360,
          h: 200,
          minW: 240,
          minH: 120,
          x: 340,
          y: 700,
          group: "POLICY",
          init: inits.dbg_policy_tl,
        },
        dbg_policy_w: {
          title: "POLICY WEIGHTS",
          w: 360,
          h: 200,
          minW: 260,
          minH: 140,
          x: 720,
          y: 700,
          group: "POLICY",
          init: inits.dbg_policy_w,
        },
      };
      return R;
    }

    return { buildRegistry, inits };
  }

  global.AgvNavUI = { install };
})(window);
