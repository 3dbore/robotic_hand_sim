import { useEffect, useRef, useState, type ReactNode, type RefObject } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { applyPose, aimPov, eeMarker, loadParts, OVERLAY, POV_H, POV_VFOV, POV_W, scene, targetMarker } from "../scene3d";
import { usePrefs, useTelemetry } from "../state";

const PRESETS = {
  Iso: { pos: [520, -620, 420], at: [0, -60, 120] },
  Top: { pos: [0, -60.1, 1100], at: [0, -60, 0] },
  Side: { pos: [900, -60, 160], at: [0, -60, 160] },
} as const;
type Preset = keyof typeof PRESETS;

/** Renders the shared scene into a canvas that follows its parent's size. */
function useRenderer(canvas: RefObject<HTMLCanvasElement | null>, setup: (r: THREE.WebGLRenderer) => (() => void) | void, frame: (r: THREE.WebGLRenderer, w: number, h: number) => void) {
  useEffect(() => {
    const c = canvas.current!;
    const r = new THREE.WebGLRenderer({ canvas: c, antialias: true });
    r.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    const cleanup = setup(r);
    let raf = 0;
    const loop = () => {
      const w = c.clientWidth, h = c.clientHeight;
      if (w && h && (c.width !== Math.floor(w * r.getPixelRatio()) || c.height !== Math.floor(h * r.getPixelRatio()))) r.setSize(w, h, false);
      if (w && h) frame(r, w, h);
      raf = requestAnimationFrame(loop);
    };
    loop();
    return () => { cancelAnimationFrame(raf); cleanup?.(); r.dispose(); };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
}

export function Viewport({ children }: { children?: ReactNode }) {
  const s = useTelemetry();
  const prefs = usePrefs();
  const canvas = useRef<HTMLCanvasElement>(null);
  const cam = useRef(new THREE.PerspectiveCamera(40, 1, 5, 6000));
  const controls = useRef<OrbitControls | null>(null);
  const [view, setView] = useState<Preset>("Iso");
  const [failed, setFailed] = useState(false);

  const aim = (p: Preset) => {
    const c = cam.current, { pos, at } = PRESETS[p];
    c.position.set(pos[0], pos[1], pos[2]);
    controls.current?.target.set(at[0], at[1], at[2]);
    controls.current?.update();
    setView(p);
  };

  useRenderer(canvas, (r) => {
    const c = cam.current;
    c.up.set(0, 0, 1);
    c.layers.enable(OVERLAY);
    const oc = new OrbitControls(c, r.domElement);
    oc.enableDamping = false;
    controls.current = oc;
    aim("Iso");
    loadParts().catch(() => setFailed(true));
    return () => oc.dispose();
  }, (r, w, h) => {
    cam.current.aspect = w / h;
    cam.current.updateProjectionMatrix();
    r.render(scene, cam.current);
  });

  useEffect(() => {
    if (!s) return;
    applyPose(prefs.mirrorBoard ? s.board_pose : s.pose);
    eeMarker.position.set(...s.ee_pos);
    targetMarker.visible = prefs.showTarget;
  }, [s, prefs.mirrorBoard, prefs.showTarget]);

  return (
    <div className="viewport">
      <canvas ref={canvas} aria-label="3D simulation of the arm" />
      <div className="title">Robotic Hand 3D Simulation</div>
      <div className="views">
        {(Object.keys(PRESETS) as Preset[]).map((p) => (
          <button key={p} className={`btn sm ${p === view ? "on" : ""}`} onClick={() => aim(p)}>{p}</button>
        ))}
      </div>
      <div className="note">sim pose = {prefs.mirrorBoard ? "board reported" : "command sent"}</div>
      {failed && <div className="err">Could not load the arm meshes from the backend</div>}
      {children}
    </div>
  );
}

/** The hand camera's view of the same scene, as the toolkit renders it. */
export function SimPov() {
  const canvas = useRef<HTMLCanvasElement>(null);
  const cam = useRef(new THREE.PerspectiveCamera(POV_VFOV, POV_W / POV_H, 5, 6000));
  useRenderer(canvas, () => { loadParts().catch(() => {}); }, (r, w, h) => {
    // letterbox to 512x320 so the framing matches the real camera
    const scale = Math.min(w / POV_W, h / POV_H);
    const vw = POV_W * scale, vh = POV_H * scale;
    r.setScissorTest(false);
    r.setClearColor(0x050505);
    r.clear();
    r.setViewport((w - vw) / 2, (h - vh) / 2, vw, vh);
    r.setScissor((w - vw) / 2, (h - vh) / 2, vw, vh);
    r.setScissorTest(true);
    aimPov(cam.current);
    r.render(scene, cam.current);
  });
  return <canvas ref={canvas} aria-label="Simulated camera view" />;
}
