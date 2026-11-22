/* Go to point: clickable top and side workspace views (as hil/widgets.py), pitch, yaw and gripper. */
import { useEffect, useRef, useState, type PointerEvent } from "react";
import { CircleX } from "lucide-react";
import { api } from "../api";
import { targetMarker } from "../scene3d";
import { axisByName, stopAll, useLastError, usePrefs } from "../state";
import type { TelemetryState } from "../types";
import { Knob, Slider, fmt } from "../ui";

type Vec3 = [number, number, number];
type Kind = "top" | "side";
const TANK = 350; // the tank box spans X ±350, Y 0..350 (scene3d)

let meta: Promise<{ reach: number; tank_z: number }> | null = null;
const sceneMeta = () => (meta ??= fetch("/api/scene/parts").then((r) => r.json()));

/** Maps between mm and canvas pixels for one view; both views keep a uniform scale. */
function frame(kind: Kind, R: number, w: number, h: number, theta: number) {
  const W = 2 * R, H = kind === "top" ? 2 * R : R + 200;
  const s = Math.min(w / W, h / H), ox = (w - W * s) / 2, oy = (h - H * s) / 2;
  const sn = Math.sin(theta), cs = Math.cos(theta);
  // top: up = -Y, right = -X. side: right = +h along the base heading, up = Z.
  const toPx = (p: Vec3): [number, number] =>
    kind === "top"
      ? [ox + (R - p[0]) * s, oy + (p[1] + R) * s]
      : [ox + (p[0] * sn - p[1] * cs + R) * s, oy + (R + 50 - p[2]) * s];
  const fromPx = (x: number, y: number, z: number): Vec3 => {
    if (kind === "top") return [R - (x - ox) / s, (y - oy) / s - R, z];
    const hh = (x - ox) / s - R;
    return [hh * sn, -hh * cs, R + 50 - (y - oy) / s];
  };
  return { s, ox, oy, W, H, toPx, fromPx, sn, cs };
}

function WsView({ kind, s, target, onPick }: { kind: Kind; s: TelemetryState; target: Vec3; onPick: (p: Vec3) => void }) {
  const prefs = usePrefs();
  const ref = useRef<HTMLCanvasElement>(null);
  const img = useRef<HTMLImageElement | null>(null);
  const [R, setR] = useState(0);
  const [tankZ, setTankZ] = useState(150);
  const [maskKey, setMaskKey] = useState<number | null>(null);
  const [tick, setTick] = useState(0);
  const theta = (axisByName(s, "Arm Base").sim_angle * Math.PI) / 180;
  const z = s.ee_pos[2];

  useEffect(() => { sceneMeta().then((m) => { setR(m.reach); setTankZ(m.tank_z); }); }, []);

  // re-fetch the reachable mask only when the slice it depends on really moved
  const want = kind === "top" ? z : (theta * 180) / Math.PI;
  const thresh = 1;
  useEffect(() => {
    if (maskKey === null || Math.abs(want - maskKey) > thresh) setMaskKey(Math.round(want * 10) / 10);
  }, [want, maskKey, thresh]);
  useEffect(() => {
    if (maskKey === null || !prefs.showWorkspace) return;
    const im = new Image();
    im.onload = () => { img.current = im; setTick((t) => t + 1); };
    im.src = `/api/workspace/${kind}.png?v=${maskKey}`;
  }, [kind, maskKey, prefs.showWorkspace]);

  useEffect(() => {
    const c = ref.current!;
    const ro = new ResizeObserver(() => setTick((t) => t + 1));
    ro.observe(c);
    return () => ro.disconnect();
  }, []);

  useEffect(() => {
    const c = ref.current!;
    const w = c.clientWidth, h = c.clientHeight, dpr = window.devicePixelRatio || 1;
    if (!w || !h || !R) return;
    c.width = w * dpr; c.height = h * dpr;
    const g = c.getContext("2d")!;
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, w, h);
    const f = frame(kind, R, w, h, theta);
    const zTop = R + 50, zBot = -150;

    if (prefs.showWorkspace && img.current) {
      g.globalAlpha = 0.9;
      g.drawImage(img.current, f.ox, f.oy, f.W * f.s, f.H * f.s);
      g.globalAlpha = 1;
    }

    // grid every 50 mm
    g.strokeStyle = "#202020"; g.lineWidth = 1;
    g.beginPath();
    for (let v = -R + (R % 50); v <= R; v += 50) {
      const x = f.ox + (v + R) * f.s;
      g.moveTo(x, f.oy); g.lineTo(x, f.oy + f.H * f.s);
    }
    const rows = kind === "top" ? [-R, R] : [zBot, zTop];
    for (let v = Math.ceil(rows[0] / 50) * 50; v <= rows[1]; v += 50) {
      const y = kind === "top" ? f.oy + (v + R) * f.s : f.oy + (zTop - v) * f.s;
      g.moveTo(f.ox, y); g.lineTo(f.ox + f.W * f.s, y);
    }
    g.stroke();

    // tank keep-out
    g.fillStyle = "rgba(255,0,4,0.18)"; g.strokeStyle = "rgba(255,0,4,0.7)";
    if (kind === "top") {
      const [x0, y0] = f.toPx([TANK, 0, 0]), [x1, y1] = f.toPx([-TANK, TANK, 0]);
      g.fillRect(x0, y0, x1 - x0, y1 - y0); g.strokeRect(x0, y0, x1 - x0, y1 - y0);
    } else if (Math.abs(f.cs) > 1e-3) {
      // the tank (Y > 0) lies on the -cosθ side of the heading line
      const far = Math.sign(-f.cs) * Math.min(R, TANK / Math.max(Math.abs(f.sn), 1e-6), TANK / Math.abs(f.cs));
      const xa = f.ox + R * f.s, xb = f.ox + (far + R) * f.s;
      const ya = f.oy + (zTop - tankZ) * f.s, yb = f.oy + zTop * f.s;
      g.fillRect(Math.min(xa, xb), ya, Math.abs(xb - xa), yb - ya);
      g.strokeRect(Math.min(xa, xb), ya, Math.abs(xb - xa), yb - ya);
    }

    if (kind === "side") {
      // floor hatch below Z 0
      const y0 = f.oy + zTop * f.s, y1 = f.oy + f.H * f.s;
      g.save(); g.beginPath(); g.rect(f.ox, y0, f.W * f.s, y1 - y0); g.clip();
      g.strokeStyle = "#3a3a3a"; g.beginPath();
      for (let x = f.ox - (y1 - y0); x < f.ox + f.W * f.s; x += 10) { g.moveTo(x, y1); g.lineTo(x + (y1 - y0), y0); }
      g.stroke(); g.restore();
      g.strokeStyle = "#686868"; g.beginPath(); g.moveTo(f.ox, y0); g.lineTo(f.ox + f.W * f.s, y0); g.stroke();
    } else {
      // heading ray from the base
      const [bx, by] = f.toPx([0, 0, 0]), [hx, hy] = f.toPx([R * f.sn, -R * f.cs, 0]);
      g.setLineDash([4, 4]); g.strokeStyle = "#686868"; g.beginPath(); g.moveTo(bx, by); g.lineTo(hx, hy); g.stroke(); g.setLineDash([]);
    }

    // arm
    const pts = s.joints.map(f.toPx);
    g.strokeStyle = "#ffffff"; g.lineWidth = 2; g.beginPath();
    pts.forEach(([x, y], i) => (i ? g.lineTo(x, y) : g.moveTo(x, y)));
    g.stroke();
    g.fillStyle = "#ffffff";
    pts.forEach(([x, y]) => { g.beginPath(); g.arc(x, y, 3, 0, 7); g.fill(); });
    const [ex, ey] = f.toPx(s.ee_pos);
    g.fillStyle = "#ff6b35"; g.beginPath(); g.arc(ex, ey, 5, 0, 7); g.fill();

    // target
    const [tx, ty] = f.toPx(target);
    g.strokeStyle = "#e2ff02"; g.lineWidth = 1.5; g.beginPath();
    g.moveTo(tx - 9, ty); g.lineTo(tx + 9, ty); g.moveTo(tx, ty - 9); g.lineTo(tx, ty + 9); g.stroke();
    g.beginPath(); g.arc(tx, ty, 5, 0, 7); g.stroke();

    // axis labels
    g.fillStyle = "#9a9a9a"; g.font = "italic 12px 'Plus Jakarta Sans', sans-serif";
    if (kind === "top") { g.fillText("-Y", f.ox + f.W * f.s / 2 + 4, f.oy + 14); g.fillText("-X", f.ox + f.W * f.s - 20, f.oy + f.H * f.s / 2 - 4); }
    else { g.fillText("Z", f.ox + R * f.s + 4, f.oy + 14); g.fillText("+R", f.ox + f.W * f.s - 22, f.oy + zTop * f.s - 4); }
  }, [s, target, R, tankZ, theta, kind, tick, prefs.showWorkspace]);

  const pick = (e: PointerEvent<HTMLCanvasElement>) => {
    if (!R) return;
    const r = e.currentTarget.getBoundingClientRect();
    const f = frame(kind, R, r.width, r.height, theta);
    const p = f.fromPx(e.clientX - r.left, e.clientY - r.top, kind === "top" ? z : 0);
    onPick(p.map((v) => Math.round(v)) as Vec3);
  };

  const hh = target[0] * Math.sin(theta) - target[1] * Math.cos(theta);
  return (
    <div className="wsview">
      <canvas ref={ref} onPointerDown={pick} aria-label={`${kind === "top" ? "Top" : "Side"} workspace view, click to set the target`} />
      <div className="title">{kind === "top" ? "Top View" : "Side View"}</div>
      <div className="read">
        {kind === "top" ? `X ${fmt(target[0], 0)} · Y ${fmt(target[1], 0)} mm` : `R ${fmt(hh, 0)} · Z ${fmt(target[2], 0)} mm`}
      </div>
    </div>
  );
}

export function Control({ s }: { s: TelemetryState }) {
  const prefs = usePrefs();
  const sim = (n: string) => axisByName(s, n).sim_angle;
  const [target, setTarget] = useState<Vec3>(() => s.ee_pos.map((v) => Math.round(v)) as Vec3);
  // start from the current orientation so a click does not force the gripper level
  const [pitch, setPitch] = useState(() => Math.round(sim("Arm A") + sim("Arm B") + sim("Wrist Pitch")));
  const [yaw, setYaw] = useState(() => Math.round(sim("Wrist Yaw")));
  const [grip, setGrip] = useState<number | null>(null);
  const [status, setStatus] = useState("");
  const err = useLastError();

  // as hil/main_window.move_to: a click on a view moves straight away
  const go = (p: Vec3 = target) => {
    setTarget(p);
    setStatus(`Moving to X ${p[0]}, Y ${p[1]}, Z ${p[2]} mm`);
    api.goToPoint(p[0], p[1], p[2], pitch, yaw);
  };

  const useCurrent = () => {
    setTarget(s.ee_pos.map((v) => Math.round(v)) as Vec3);
    setPitch(Math.round(sim("Arm A") + sim("Arm B") + sim("Wrist Pitch")));
    setYaw(Math.round(sim("Wrist Yaw")));
  };

  useEffect(() => {
    targetMarker.position.set(...target);
    targetMarker.visible = prefs.showTarget;
  }, [target, prefs.showTarget]);

  return (
    <div className="control">
      <div className="phead">
        <h2>Control</h2>
        <span className={err ? "sub bad" : "sub muted"}>{err || status || "Click a view to move there"}</span>
        <div className="end">
          <button className="btn" onClick={useCurrent}>Use current</button>
          <button className="btn danger" onClick={stopAll}><CircleX size={16} />Emergency stop</button>
        </div>
      </div>
      <div className="ctlgrid">
        <WsView kind="top" s={s} target={target} onPick={go} />
        <WsView kind="side" s={s} target={target} onPick={go} />
        <div className="vbox">
          <i>Pitch</i>
          <Slider vertical label="Target pitch" min={-90} max={90} value={pitch} band={[-90, 90]} tick={0} onCommit={(v) => { setPitch(v); api.setWrist(v, undefined); }} onChange={setPitch} />
          <span className="big">{fmt(pitch, 0)}°</span>
        </div>
        <div className="vcol">
          <div className="vbox" style={{ flex: "none" }}>
            <i>Yaw</i>
            <Knob value={yaw} range={90} onCommit={(v) => { setYaw(v); api.setWrist(undefined, v); }}><span /></Knob>
          </div>
          <div className="vbox" style={{ gap: "var(--sp-xs)" }}>
            <i>Gripper</i>
            <Slider label="Gripper opening" min={0} max={100} value={grip ?? s.gripper_percent} band={[0, grip ?? s.gripper_percent]}
              onChange={setGrip} onCommit={(v) => { setGrip(null); api.setGripper(v); }} />
            <span className="big">{fmt(grip ?? s.gripper_percent, 0)}%</span>
          </div>
        </div>
      </div>
    </div>
  );
}
