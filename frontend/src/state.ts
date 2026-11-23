/* Telemetry and per-browser preferences as React hooks. */
import { useEffect, useState } from "react";
import { api } from "./api";
import type { AxisState, TelemetryState } from "./types";

export function useTelemetry() {
  const [s, set] = useState<TelemetryState | null>(api.currentState);
  useEffect(() => api.subscribe(set), []);
  return s;
}

/** Last refusal or error from the backend, cleared after a few seconds. */
export function useLastError(ms = 5000) {
  const [msg, set] = useState("");
  useEffect(() => {
    let t = 0;
    const off = api.onError((m) => { set(m); clearTimeout(t); t = window.setTimeout(() => set(""), ms); });
    return () => { off(); clearTimeout(t); };
  }, [ms]);
  return msg;
}

export interface Prefs {
  jog: number;
  showWorkspace: boolean;
  showTarget: boolean;
  mirrorBoard: boolean;
  releaseOnEsc: boolean;
  autoReconnect: boolean;
  tagOnSave: boolean;
  /** opacity of the gripper targeting line and grip box, 0..1 */
  guideOpacity: number;
}

const DEFAULTS: Prefs = { jog: 2, showWorkspace: true, showTarget: true, mirrorBoard: false, releaseOnEsc: true, autoReconnect: true, tagOnSave: true, guideOpacity: 0.5 };
const KEY = "elios-hil-prefs";

function load(): Prefs {
  try {
    return { ...DEFAULTS, ...JSON.parse(localStorage.getItem(KEY) || "{}") };
  } catch {
    return DEFAULTS;
  }
}

let prefs = load();
const subs = new Set<(p: Prefs) => void>();

export function setPrefs(patch: Partial<Prefs>) {
  prefs = { ...prefs, ...patch };
  try { localStorage.setItem(KEY, JSON.stringify(prefs)); } catch { /* private mode: keep in memory */ }
  subs.forEach((f) => f(prefs));
}

export function usePrefs(): Prefs {
  const [p, set] = useState(prefs);
  useEffect(() => { subs.add(set); return () => { subs.delete(set); }; }, []);
  return p;
}

export const prefsNow = () => prefs;

/** Axes in UI order: Base, Arm A, Arm B, Pitch, Yaw, Gripper (F5..F0). */
export const displayAxes = (axes: AxisState[]) => axes.map((a, i) => ({ ...a, f: `F${i}` })).reverse();

export const axisByName = (s: TelemetryState, name: string) => s.axes.find((a) => a.name === name)!;

/** Stop the arm, and release the motors when the preference says so (Esc and the Stop button). */
export function stopAll() {
  api.stop();
  api.stopSequence();
  if (prefs.releaseOnEsc) api.toggleMotors(false);
}

export function goHome(s: TelemetryState) {
  api.send("go_pose", { servo: Object.fromEntries(s.axes.map((a) => [a.name, a.home])) });
}
